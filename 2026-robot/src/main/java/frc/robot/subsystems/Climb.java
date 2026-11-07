package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.EnergyReporter;
import frc.robot.lib.LoggedTalonFXS;
import frc.robot.lib.Tunable;

public final class Climb extends SubsystemBase implements EnergyReporter {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kP = 5;
  private static final double __kD = 0;

  private static final double __kPd = 3;
  private static final double __kDd = 0;

  private static final double __kPositionExtended = 8.0; // inches
  private static final double __kPositionEngaged = 2.0;
  private static final double __kPositionRetracted = 0;
  private static final double __kPositionManual = 0;

  // 2.73 is effective spool diameter in inches
  private static final double kConversionFactor = 36.0 / (Math.PI * 2.73);

  // motion profile for lowering the robot
  private static final double __kMaxVelocity = kConversionFactor * 6.0; // rot/sec (6 in/sec)
  private static final double __kMaxAcceleration =
      __kMaxVelocity / 0.2; // rot/sec^2 (200ms to max velocity)

  @Logged(name = "Motor", importance = Importance.CRITICAL)
  private final LoggedTalonFXS motor;

  @Logged(name = "Setpoint", importance = Importance.CRITICAL)
  private Distance setpoint = Inches.zero();

  private final PositionVoltage control;
  private final DynamicMotionMagicVoltage controlProfiled;

  // indicates that the subsystem has been initialized by zeroing the position of the climb
  private boolean initialized = false;

  private static enum State {
    kNeutral,
    kEngaging,
    kEngaged,
  };

  private State state = State.kNeutral;

  // supports auto-retract once the robot clears the tower zone, after engaging the climb
  private boolean disengagedButNotRetracted = false;

  private static final double kStatorCurrentLimit = 100.0;
  private static final double kTolerance = 0.5; // inches

  public Climb(CANBus canbus) {
    motor =
        new LoggedTalonFXS(
            Constants.CanivoreID.MOTOR_CLIMB,
            canbus,
            (config) -> {
              config.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
              config.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
              config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
              config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
              config.MotorOutput.ControlTimesyncFreqHz = 100;

              config.Slot0.kP = __kP;
              config.Slot0.kD = __kD;

              config.Slot1.kP = __kPd;
              config.Slot1.kD = __kDd;

              config.MotionMagic.MotionMagicCruiseVelocity = __kMaxVelocity;
              config.MotionMagic.MotionMagicAcceleration = __kMaxAcceleration;
            });

    control =
        new PositionVoltage(0)
            .withSlot(0)
            .withUseTimesync(true)
            .withUpdateFreqHz(0)
            .withEnableFOC(true);

    controlProfiled =
        new DynamicMotionMagicVoltage(0, __kMaxVelocity, __kMaxAcceleration)
            .withSlot(1)
            .withUseTimesync(true)
            .withUpdateFreqHz(0)
            .withEnableFOC(true);

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Climb", motor);
    Tunable.bind1("Climb/Profiled", motor);
    Tunable.bindMotionProfile("Climb/Profiled", motor);
  }

  @Override
  public void periodic() {
    // run initialization sequence to home the climb
    if (!initialized) {
      motor.set(-0.05);

      // climb is zeroed against a hard stop
      //
      // note that since we typically require very high stator current for actual climb,
      // we use much lower threhsold here to detect stall during initialization sequence
      if (motor.isStalled(10)) {
        motor.stopMotor();
        motor.resetPosition(0);
        initialized = true;
      }
    } else {
      if (kPositionManualOverride.get()) {
        setpoint = Inches.of(kPositionManual.get());
      }

      // use profiled motion if the climb is engaged to lower the robot;
      // use faster profiled motion to extend the climb;
      // otherwise, use simple PID to jump to the setpoint
      final var position = setpoint.in(Inches) * kConversionFactor;

      if (state == State.kEngaged) {
        controlProfiled.Velocity = __kMaxVelocity;
        motor.setControl(controlProfiled.withPosition(position));
      } else if (getPosition().lt(setpoint) && state != State.kEngaging) {
        controlProfiled.Velocity = __kMaxVelocity * 1.4;
        motor.setControl(controlProfiled.withPosition(position));
      } else if (setpoint.in(Inches) == 0 && isAtSetpoint()) {
        motor.stopMotor();
      } else {
        motor.setControl(control.withPosition(position));
      }
    }

    // reset "engaged" flag once the hook is fully extended
    switch (state) {
      case kEngaged:
        final var positionExtended = kPositionExtended.get();
        if (MathUtil.isNear(setpoint.in(Inches), positionExtended, kTolerance)
            && getPosition().in(Inches) >= positionExtended - kTolerance) {
          state = State.kNeutral;
          disengagedButNotRetracted = true;
        }
        break;

      case kEngaging:
        if (MathUtil.isNear(getPosition().in(Inches), kPositionEngaged.get(), kTolerance)) {
          state = State.kEngaged;
        }
        break;

      default:
        // ignore
        break;
    }
  }

  /** Determines whether the subsystem is at its commanded setpoint. */
  @Logged(name = "AtSetpoint", importance = Importance.CRITICAL)
  public boolean isAtSetpoint() {
    return MathUtil.isNear(setpoint.in(Inches), getPosition().in(Inches), kTolerance);
  }

  @Logged(name = "Position", importance = Importance.CRITICAL)
  public Distance getPosition() {
    return Inches.of(motor.getMotorPosition().in(Rotations) / kConversionFactor);
  }

  @Logged(name = "Initialized", importance = Importance.CRITICAL)
  public boolean isInitialized() {
    return initialized;
  }

  @Logged(name = "Ready", importance = Importance.CRITICAL)
  public boolean isReady() {
    return MathUtil.isNear(kPositionExtended.get(), getPosition().in(Inches), kTolerance);
  }

  @Logged(name = "Engaged", importance = Importance.CRITICAL)
  public boolean isEngaged() {
    return state == State.kEngaged;
  }

  @Logged(name = "DisengagedButNotRetracted", importance = Importance.CRITICAL)
  public boolean isDisengagedButNotRetracted() {
    return disengagedButNotRetracted;
  }

  /** Extends the hook. */
  public Command extend() {
    return runOnce(
            () -> {
              setpoint = Inches.of(kPositionExtended.get());
            })
        .withName("ClimbExtend");
  }

  /** Retracts the hook. */
  public Command retract() {
    return runOnce(
            () -> {
              setpoint = Inches.of(kPositionRetracted.get());
              disengagedButNotRetracted = false;
            })
        .withName("ClimbRetract");
  }

  /** Engages the climb by rapdidly retracting the hook to a pre-defined position. */
  public Command engage() {
    return runOnce(
            () -> {
              state = State.kEngaging;
              setpoint = Inches.of(kPositionEngaged.get());
            })
        .withName("ClimbEngage");
  }

  /** Disengages the climb by lowering the robot to the ground in a controllable manner. */
  public Command disengage() {
    return runOnce(
            () -> {
              // offset by 0.5" to reduce the chance of unspooling
              // when the hook sticks to the climb rung
              setpoint = Inches.of(kPositionExtended.get() - 0.5);
            })
        .withName("ClimbDisengage")
        // a "hack" that resets the "engaged" flag after a timeout in case we never fully
        // reached the setpoint somehow, note that we do not require this subsystem in
        // the command being scheduled
        //
        // this way we can quickly extend/retract the hook manually, however it opens up
        // a possibility of a race, if the climb is re-engaged in the meantime,
        // yet this specific scenario is extremely rare in practice
        .andThen(
            new ScheduleCommand(
                Commands.sequence(
                    Commands.waitSeconds(2),
                    Commands.runOnce(
                        () -> {
                          state = State.kNeutral;
                          if (getPosition().in(Inches) >= kPositionRetracted.get() + kTolerance) {
                            disengagedButNotRetracted = true;
                          }
                        }))));
  }

  @Override
  public double getTotalSupplyCurrent() {
    return Math.abs(motor.getSupplyCurrentAmps());
  }

  /** Runs self-check. */
  public Command selfCheck() {
    return Commands.sequence(
        Commands.print("Climb self-check started"),
        Commands.waitUntil(this::isInitialized),
        Commands.print("Climb initialized"),
        extend(),
        Commands.waitTime(Seconds.of(2)),
        retract(),
        Commands.waitTime(Seconds.of(1)),
        Commands.print("Climb self-check completed"));
  }

  // spotless:off
  private static Tunable.DoubleValue kPositionExtended = Tunable.value("Climb/Extended", __kPositionExtended);
  private static Tunable.DoubleValue kPositionRetracted = Tunable.value("Climb/Retracted", __kPositionRetracted);
  private static Tunable.DoubleValue kPositionEngaged = Tunable.value("Climb/Engaged", __kPositionEngaged);
  private static Tunable.DoubleValue kPositionManual = Tunable.value("Climb/Manual", __kPositionManual);
  private static Tunable.BooleanValue kPositionManualOverride = Tunable.value("Climb/ManualOverride", false);
  // spotless:on
}
