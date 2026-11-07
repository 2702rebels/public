package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.EnergyReporter;
import frc.robot.lib.LoggedTalonFX;
import frc.robot.lib.Misc;
import frc.robot.lib.PhoenixUtil;
import frc.robot.lib.Tunable;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public final class Shooter extends SubsystemBase implements EnergyReporter {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kFlywheelS = 6.85;
  private static final double __kFlywheelV = 0.023;
  private static final double __kFlywheelP = 10;
  private static final double __kFlywheelD = 0;
  private static final double __kHoodG = 0;
  private static final double __kHoodS = 0;
  private static final double __kHoodP = 15;
  private static final double __kHoodD = 0.5;

  private static final double __kHoodParkAngle = 5;
  private static final double __kHoodDefaultAngle = 5;

  private static final double __kHoodManual = 15;
  private static final double __kFlywheelManual = 3700; // RPMs
  private static final double __kFlywheelTrim = 0; // RPMs

  @Logged(name = "Flywheel/Leader", importance = Importance.CRITICAL)
  private final LoggedTalonFX flywheelMotorLeader;

  @Logged(name = "Flywheel/Follower", importance = Importance.CRITICAL)
  private final LoggedTalonFX flywheelMotorFollower;

  @Logged(name = "Hood", importance = Importance.CRITICAL)
  private final LoggedTalonFX hoodMotor;

  @Logged(name = "Setpoint/Flywheel", importance = Importance.CRITICAL)
  private double flywheelSetpoint = 0;

  @Logged(name = "Setpoint/Hood", importance = Importance.CRITICAL)
  private Angle hoodSetpoint = Degrees.zero();

  private final Follower followerControl;
  private final VelocityTorqueCurrentFOC flywheelControl;
  private final PositionTorqueCurrentFOC hoodControl;

  @Logged(name = "Enabled", importance = Importance.CRITICAL)
  private boolean enabled = false;

  // indicates that the subsystem has been initialized by zeroing the position of the hood
  private boolean initialized = false;
  private boolean hoodParked;

  // external parameters supplier in tracking mode
  private Supplier<Optional<Pair<Double, Double>>> trackingParametersSupplier =
      () -> Optional.empty();

  private static final double kFlywheelStatorCurrentLimit = 60.0;
  private static final double kHoodStatorCurrentLimit = 20.0;

  private static final double kHoodPositionMin = 0.0; // degrees
  private static final double KHoodPositionMax = 25.0; // degrees
  private static final double kHoodPositionTolerance = 1.0; // degrees
  private static final double kHoodConversionFactor = 108.0;

  private static final double kFlywheelVelocityMax = 6500.0;
  private static final double kFlywheelVelocityTolerance = 100.0; // RPMs
  private static final double kFlywheelConversionFactor = 15.0 / 18.0;

  /** Convert hood angular position to motor position in rotations. */
  private static double toHoodMotorPosition(Angle value) {
    return value.in(Rotations) * kHoodConversionFactor;
  }

  @Logged(name = "MasterOverride", importance = Importance.CRITICAL)
  public boolean masterOverride = false;

  private final BooleanSupplier isAtSetpointDebounced;

  public Shooter(CANBus canbus) {
    flywheelMotorLeader =
        new LoggedTalonFX(
            Constants.CanivoreID.MOTOR_SHOOTER_FLYWHEEL_LEADER,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kFlywheelStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
              config.MotorOutput.ControlTimesyncFreqHz = 250;

              config.Slot0.kS = __kFlywheelS;
              config.Slot0.kV = __kFlywheelV;
              config.Slot0.kP = __kFlywheelP;
              config.Slot0.kD = __kFlywheelD;
            });

    flywheelMotorFollower =
        new LoggedTalonFX(
            Constants.CanivoreID.MOTOR_SHOOTER_FLYWHEEL_FOLLOWER,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kFlywheelStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
              config.MotorOutput.ControlTimesyncFreqHz = 250;
            });

    hoodMotor =
        new LoggedTalonFX(
            Constants.CanivoreID.MOTOR_SHOOTER_HOOD,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kHoodStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
              config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
              config.MotorOutput.ControlTimesyncFreqHz = 100;

              config.Slot0.kG = __kHoodG;
              config.Slot0.kS = __kHoodS;
              config.Slot0.kP = __kHoodP;
              config.Slot0.kD = __kHoodD;
              config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
            });

    // follow leader's output (inverted)
    followerControl =
        new Follower(flywheelMotorLeader.getDeviceID(), MotorAlignmentValue.Opposed)
            .withUpdateFreqHz(250);

    PhoenixUtil.run(
        "Shooter Follow Leader",
        flywheelMotorFollower.getDeviceID(),
        () -> flywheelMotorFollower.setControl(followerControl));

    flywheelControl =
        new VelocityTorqueCurrentFOC(0).withSlot(0).withUseTimesync(true).withUpdateFreqHz(0);

    hoodControl =
        new PositionTorqueCurrentFOC(toHoodMotorPosition(Degrees.of(KHoodPositionMax)))
            .withSlot(0)
            .withUseTimesync(true)
            .withUpdateFreqHz(0);

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Flywheel", flywheelMotorLeader);
    Tunable.bind("Hood", hoodMotor);

    hoodMotor.resetPosition(0);

    isAtSetpointDebounced = Misc.debounce(this::isAtSetpoint, 0.150, DebounceType.kFalling);
  }

  @Override
  public void periodic() {
    final var parameters = trackingParametersSupplier.get();

    // run initialization sequence to home the hood
    if (!initialized) {
      hoodMotor.set(-0.1);

      // hood is zeroed against a hard stop
      if (hoodMotor.isStalled()) {
        hoodMotor.stopMotor();
        hoodMotor.resetPosition(0);
        initialized = true;
      }
    } else {
      if (masterOverride || kHoodManualOverride.get()) {
        setAngle(kHoodManual.get());
      } else {
        if (parameters.isPresent()) {
          setAngle(parameters.get().getFirst());
        } else {
          setAngle(kHoodDefaultAngle.get());
        }
      }

      final var hoodSetpointActual = hoodParked ? Degrees.of(kHoodParkAngle.get()) : hoodSetpoint;
      hoodMotor.setControl(hoodControl.withPosition(toHoodMotorPosition(hoodSetpointActual)));
    }

    if (enabled) {
      if (masterOverride || kFlywheelManualOverride.get()) {
        setVelocity(kFlywheelManual.get());
      } else {
        if (parameters.isPresent()) {
          setVelocity(parameters.get().getSecond() + kFlywheelTrim.get());
        } else {
          setVelocity(0);
        }
      }
    } else {
      setVelocity(0);
    }

    if (flywheelSetpoint > 0) {
      if (flywheelSetpoint >= kFlywheelVelocityMax) {
        flywheelMotorLeader.set(1.0); // max output
      } else {
        flywheelMotorLeader.setControl(
            flywheelControl.withVelocity(
                RotationsPerSecond.convertFrom(flywheelSetpoint, RPM) * kFlywheelConversionFactor));
      }
    } else {
      flywheelMotorLeader.stopMotor();
    }
  }

  private void setAngle(double valueInDegrees) {
    hoodSetpoint = Degrees.of(MathUtil.clamp(valueInDegrees, kHoodPositionMin, KHoodPositionMax));
  }

  private void setVelocity(double valueInRPMs) {
    flywheelSetpoint = MathUtil.clamp(valueInRPMs, 0, kFlywheelVelocityMax);
  }

  @Logged(name = "Initialized", importance = Importance.CRITICAL)
  public boolean isInitialized() {
    return initialized;
  }

  @Logged(name = "HoodParked", importance = Importance.CRITICAL)
  public boolean isHoodParked() {
    return hoodParked;
  }

  /** Determines whether the subsystem is at its commanded setpoint. */
  @Logged(name = "AtSetpoint", importance = Importance.CRITICAL)
  public boolean isAtSetpoint() {
    return MathUtil.isNear(hoodSetpoint.in(Degrees), getAngle().in(Degrees), kHoodPositionTolerance)
        && MathUtil.isNear(flywheelSetpoint, getVelocity(), kFlywheelVelocityTolerance);
  }

  @Logged(name = "IsReadyPassing", importance = Importance.CRITICAL)
  public boolean isReadyPassing() {
    return flywheelSetpoint > 0 && getVelocity() >= flywheelSetpoint * 0.4;
  }

  @Logged(name = "IsReady", importance = Importance.CRITICAL)
  public boolean isReady() {
    return flywheelSetpoint > 0 && isAtSetpointDebounced.getAsBoolean();
  }

  @Logged(name = "Angle", importance = Importance.CRITICAL)
  public Angle getAngle() {
    return hoodMotor.getMotorPosition().div(kHoodConversionFactor);
  }

  @Logged(name = "Velocity", importance = Importance.CRITICAL)
  public double getVelocity() {
    return flywheelMotorLeader.getMotorVelocity().in(RPM) / kFlywheelConversionFactor;
  }

  /** Enables the subsystem. */
  public Command enable() {
    return runEnd(() -> enabled = true, () -> enabled = false).withName("ShooterEnable");
  }

  /** Parks hood for driving under the trench. */
  public Command parkHood() {
    return runEnd(() -> hoodParked = true, () -> hoodParked = false).withName("ParkHood");
  }

  public void setTrackingSupplier(Supplier<Optional<Pair<Double, Double>>> supplier) {
    trackingParametersSupplier = supplier;
  }

  @Override
  public double getTotalSupplyCurrent() {
    return Math.abs(flywheelMotorLeader.getSupplyCurrentAmps())
        + Math.abs(flywheelMotorFollower.getSupplyCurrentAmps())
        + Math.abs(hoodMotor.getSupplyCurrentAmps());
  }

  /** Runs self-check. */
  public Command selfCheck() {
    return Commands.sequence(
            Commands.print("Shooter self-check started"),
            Commands.waitUntil(this::isInitialized),
            Commands.print("Shooter initialized"),
            // hood sequence
            runOnce(() -> kHoodManualOverride.set(true)),
            Commands.print("Hood to 5"),
            run(() -> kHoodManual.set(5)).withTimeout(Seconds.of(2)),
            Commands.print("Hood to 15"),
            run(() -> kHoodManual.set(15)).withTimeout(Seconds.of(2)),
            Commands.print("Hood to 25"),
            run(() -> kHoodManual.set(25)).withTimeout(Seconds.of(2)),
            Commands.print("Hood parked"),
            parkHood().withTimeout(Seconds.of(2)),
            run(() -> kHoodManual.set(0)).withTimeout(Seconds.of(2)),
            runOnce(() -> kHoodManualOverride.set(false)),
            // flywheel sequence
            runOnce(() -> kFlywheelManualOverride.set(true)),
            runOnce(() -> enabled = true),
            Commands.print("Flywheel to 3000"),
            run(() -> kFlywheelManual.set(3000)).withTimeout(Seconds.of(2)),
            Commands.print("Flywheel to 4000"),
            run(() -> kFlywheelManual.set(4000)).withTimeout(Seconds.of(2)),
            Commands.print("Flywheel to 5000"),
            run(() -> kFlywheelManual.set(5000)).withTimeout(Seconds.of(2)),
            runOnce(() -> kFlywheelManualOverride.set(false)),
            runOnce(() -> enabled = false),
            Commands.print("Shooter self-check completed"))
        .withName("ShooterSelfCheck");
  }

  // spotless:off
  private static Tunable.DoubleValue kHoodParkAngle = Tunable.value("Hood/ParkAngle", __kHoodParkAngle);
  private static Tunable.DoubleValue kHoodDefaultAngle = Tunable.value("Hood/DefaultAngle", __kHoodDefaultAngle);
  private static Tunable.DoubleValue kHoodManual = Tunable.value("Hood/Manual", __kHoodManual, true);
  private static Tunable.BooleanValue kHoodManualOverride = Tunable.value("Hood/ManualOverride", false, true);

  private static Tunable.DoubleValue kFlywheelManual = Tunable.value("Flywheel/Manual", __kFlywheelManual, true);
  private static Tunable.BooleanValue kFlywheelManualOverride = Tunable.value("Flywheel/ManualOverride", false, true);
  private static Tunable.DoubleValue kFlywheelTrim = Tunable.value("Flywheel/Trim", __kFlywheelTrim, true);
  // spotless:on
}
