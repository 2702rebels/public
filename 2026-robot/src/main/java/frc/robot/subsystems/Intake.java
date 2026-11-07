package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.EnergyReporter;
import frc.robot.lib.LoggedTalonFX;
import frc.robot.lib.PhoenixUtil;
import frc.robot.lib.Tunable;
import java.util.function.BooleanSupplier;

public final class Intake extends SubsystemBase implements EnergyReporter {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kRollerS = 2.5;
  private static final double __kRollerV = 0.016;
  private static final double __kRollerP = 5;
  private static final double __kRollerD = 0;

  // roller SPID values for VoltageVelocity control w/o FOC (in case we need it)
  // S = 0.18, V = 0.11, P = 7, D = 0

  private static final double __kDeployHoldS = 0;
  private static final double __kDeployHoldP = 10;
  private static final double __kDeployHoldD = 0;

  private static final double __kDeployMoveS = 0;
  private static final double __kDeployMoveP = 5;
  private static final double __kDeployMoveI = 0;
  private static final double __kDeployMoveD = 0.2;

  private static final double __kRollerVelocity = 2750; // RPMs
  private static final double __kRetractedPosition = 1; // inches
  private static final double __kExtendedPosition = 11; // inches
  private static final double __kAgitatedPosition = 1; // inches

  @Logged(name = "Roller/Leader", importance = Importance.CRITICAL)
  private final LoggedTalonFX rollerMotorLeader;

  @Logged(name = "Roller/Follower", importance = Importance.CRITICAL)
  private final LoggedTalonFX rollerMotorFollower;

  @Logged(name = "Deploy", importance = Importance.CRITICAL)
  private final LoggedTalonFX deployMotor;

  @Logged(name = "Setpoint/Deploy", importance = Importance.CRITICAL)
  private double deploySetpoint = 0;

  @Logged(name = "Setpoint/Roller", importance = Importance.CRITICAL)
  private double rollerSetpoint = 0;

  private final Follower followerControl;
  private final VelocityTorqueCurrentFOC rollerControl;
  private final PositionTorqueCurrentFOC deployHoldControl;
  private final PositionTorqueCurrentFOC deployMoveControl;

  @Logged(name = "Enabled", importance = Importance.CRITICAL)
  private boolean enabled = false;

  // indicates that the subsystem has been initialized by zeroing the position of the intake
  private boolean initialized = false;

  private static enum State {
    kHolding,
    kExtending,
    kRetracting,
    kAgitating,
  };

  @Logged(name = "State", importance = Importance.CRITICAL)
  public State state = State.kHolding;

  private boolean deployed;
  private boolean reversed;

  // debounced stalling condition on deploy motor
  private BooleanSupplier deployMotorStalled;

  private static final double kRollerStatorCurrentLimit = 80.0;
  private static final double kDeployStatorCurrentLimit = 40.0;

  private static final double kRollerConversionFactor = 36.0 / 18.0;
  private static final double kDeployConversionFactor = 10.0 / (1.6 * Math.PI);
  private static final double kRollerTolerance = 100.0; // RPMs
  private static final double kDeployTolerance = 0.5; // inches
  private static final double kRollerMinPosition = 2.0; // inches

  public Intake(CANBus canbus) {
    rollerMotorLeader =
        new LoggedTalonFX(
            Constants.CanivoreID.MOTOR_INTAKE_ROLLER_LEADER,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kRollerStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
              config.MotorOutput.ControlTimesyncFreqHz = 100;

              config.Slot0.kS = __kRollerS;
              config.Slot0.kV = __kRollerV;
              config.Slot0.kP = __kRollerP;
              config.Slot0.kD = __kRollerD;
            });

    rollerMotorFollower =
        new LoggedTalonFX(
            Constants.CanivoreID.MOTOR_INTAKE_ROLLER_FOLLOWER,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kRollerStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
              config.MotorOutput.ControlTimesyncFreqHz = 250;
            });

    deployMotor =
        new LoggedTalonFX(
            Constants.CanivoreID.MOTOR_INTAKE_DEPLOY,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kDeployStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
              config.MotorOutput.ControlTimesyncFreqHz = 250;

              config.Slot0.kS = __kDeployHoldS;
              config.Slot0.kP = __kDeployHoldP;
              config.Slot0.kD = __kDeployHoldD;
              config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

              config.Slot1.kS = __kDeployMoveS;
              config.Slot1.kP = __kDeployMoveP;
              config.Slot1.kI = __kDeployMoveI;
              config.Slot1.kD = __kDeployMoveD;
            });

    // follow leader's output (inverted)
    followerControl =
        new Follower(rollerMotorLeader.getDeviceID(), MotorAlignmentValue.Opposed)
            .withUpdateFreqHz(250);

    PhoenixUtil.run(
        "Roller Follow Leader",
        rollerMotorFollower.getDeviceID(),
        () -> rollerMotorFollower.setControl(followerControl));

    rollerControl =
        new VelocityTorqueCurrentFOC(0).withSlot(0).withUseTimesync(true).withUpdateFreqHz(0);

    deployHoldControl =
        new PositionTorqueCurrentFOC(0).withSlot(0).withUseTimesync(true).withUpdateFreqHz(0);

    deployMoveControl =
        new PositionTorqueCurrentFOC(0).withSlot(1).withUseTimesync(true).withUpdateFreqHz(0);

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Roller", rollerMotorLeader);
    Tunable.bind("Deploy/Hold", deployMotor);
    Tunable.bind1("Deploy/Move", deployMotor);

    // zero position based on the initial configuration
    deployMotor.resetPosition(0);

    // debounce to give the intake time to fully zero itself
    deployMotorStalled = () -> deployMotor.isStalled(30);
  }

  @Override
  public void periodic() {
    final var position = getPosition();

    // run initialization sequence to home the intake
    if (!initialized) {
      deployMotor.set(-0.2);

      // intake is zeroed against a hard stop
      if (deployMotorStalled.getAsBoolean()) {
        deployMotor.stopMotor();
        deployMotor.resetPosition(0);
        initialized = true;
      }
    } else {
      if (state == State.kAgitating) {
        if (position > kAgitatedPosition.get()) {
          deployMotor.set(-0.2);
        } else {
          deployMotor.setControl(
              deployHoldControl.withPosition(kAgitatedPosition.get() * kDeployConversionFactor));
        }
      } else {
        ControlRequest controlRequest;
        switch (state) {
          case kExtending:
          case kRetracting:
            controlRequest =
                deployMoveControl.withPosition(deploySetpoint * kDeployConversionFactor);
            break;

          default:
            controlRequest =
                deployHoldControl.withPosition(deploySetpoint * kDeployConversionFactor);
            break;
        }

        // switch to holding pattern once the position is reached
        if (state != State.kHolding
            && MathUtil.isNear(deploySetpoint, position, kDeployTolerance)) {
          deployed = state == State.kExtending;
          state = State.kHolding;
        }

        deployMotor.setControl(controlRequest);
      }

      // reset zero when intake reading negative position
      if (position < 0) {
        deployMotor.resetPosition(0);
      }
    }

    if (reversed) {
      rollerMotorLeader.set(-0.2);
    } else {
      // pause intaking when intake is almost all the way in
      if (enabled && position >= kRollerMinPosition) {
        rollerSetpoint = kRollerVelocity.get();
        rollerMotorLeader.setControl(
            rollerControl.withVelocity(
                RotationsPerSecond.convertFrom(rollerSetpoint, RPM) * kRollerConversionFactor));
      } else {
        rollerMotorLeader.stopMotor();
      }
    }
  }

  @Logged(name = "Initialized", importance = Importance.CRITICAL)
  public boolean isInitialized() {
    return initialized;
  }

  @Logged(name = "Deployed", importance = Importance.CRITICAL)
  public boolean isDeployed() {
    return deployed;
  }

  /** Determines whether the subsystem is at its nominal state. */
  @Logged(name = "Nominal", importance = Importance.CRITICAL)
  public boolean isNominal() {
    return MathUtil.isNear(deploySetpoint, getPosition(), kDeployTolerance)
        && MathUtil.isNear(rollerSetpoint, getVelocity(), kRollerTolerance);
  }

  @Logged(name = "Velocity", importance = Importance.CRITICAL)
  public double getVelocity() {
    return rollerMotorLeader.getMotorVelocity().in(RPM) / kRollerConversionFactor;
  }

  @Logged(name = "Position", importance = Importance.CRITICAL)
  public double getPosition() {
    return deployMotor.getMotorPosition().in(Rotations) / kDeployConversionFactor;
  }

  /** Disables the subsystem. */
  public Command disable() {
    return runOnce(() -> enabled = false).withName("IntakeDisable");
  }

  /** Enables the subsystem. */
  public Command enable() {
    return runOnce(() -> enabled = true).withName("IntakeEnable");
  }

  /** Agitates the intake. */
  public Command agitate() {
    return runEnd(
            () -> state = State.kAgitating,
            () -> {
              state = State.kExtending;
              deploySetpoint = kExtendedPosition.get();
            })
        .withName("IntakeAgitate");
  }

  /** Extends the intake. */
  public Command extend() {
    return runOnce(
            () -> {
              state = State.kExtending;
              deploySetpoint = kExtendedPosition.get();
            })
        .withName("IntakeExtend");
  }

  /** Retracts the intake. */
  public Command retract() {
    return runOnce(
            () -> {
              state = State.kRetracting;
              deploySetpoint = kRetractedPosition.get();
            })
        .withName("IntakeRetract");
  }

  /** Reverses the roller to regurgitate the balls. */
  public Command burp() {
    // DO NOT REQUIRE THIS SUBSYSTEM
    return Commands.runEnd(() -> reversed = true, () -> reversed = false).withName("IntakeBurp");
  }

  @Override
  public double getTotalSupplyCurrent() {
    return Math.abs(deployMotor.getSupplyCurrentAmps())
        + Math.abs(rollerMotorLeader.getSupplyCurrentAmps())
        + +Math.abs(rollerMotorFollower.getSupplyCurrentAmps());
  }

  /** Runs self-check. */
  public Command selfCheck() {
    return Commands.sequence(
            Commands.print("Intake self-check started"),
            Commands.waitUntil(this::isInitialized),
            Commands.print("Intake initialized"),
            extend(),
            enable(),
            Commands.waitTime(Seconds.of(2)),
            agitate().withTimeout(Seconds.of(3)),
            Commands.waitTime(Seconds.of(0.5)),
            disable(),
            Commands.print("Intake self-check completed"))
        .withName("IntakeSelfCheck");
  }

  // spotless:off
  private static Tunable.DoubleValue kRollerVelocity = Tunable.value("Roller/Velocity", __kRollerVelocity);
  private static Tunable.DoubleValue kExtendedPosition = Tunable.value("Deploy/Extended", __kExtendedPosition);
  private static Tunable.DoubleValue kRetractedPosition = Tunable.value("Deploy/Retracted", __kRetractedPosition);
  private static Tunable.DoubleValue kAgitatedPosition = Tunable.value("Deploy/Agitated", __kAgitatedPosition);
  // spotless:on
}
