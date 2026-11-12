package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.CanID;
import frc.robot.Constants;

/** Drive subsystem related constants. All measured numeric values are in meters and/or radians. */
public final class DriveConstants {
  /**
   * The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T. This
   * changes the drive speed of the module (a pinion gear with more teeth will result in a robot
   * that drives faster).
   */
  private static final double driveMotorPinionTeeth = 14.0;

  public static final int driveEncoderMeasurementPeriod = 10;
  public static final int driveEncoderMeasurementDepth = 4;
  public static final int turnEncoderMeasurementDepth = 2;

  /**
   * Drive motor configuration
   *
   * <p>Reduction = 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
   * teeth on the bevel pinion.
   */
  private static final MotorConfig driveMotorConfig =
      new MotorConfig(
          (45.0 * 22.0) / (driveMotorPinionTeeth * 15.0),
          5676.0, // NEO free speed is 5676 RPM
          60, // 60A
          12 // 12V
          );

  /**
   * Turn motor configuration
   *
   * <p>Reduction = azimuth ratio from REV specification https://www.revrobotics.com/rev-21-3005/
   */
  private static final MotorConfig turnMotorConfig =
      new MotorConfig(
          9424.0 / 203.0,
          11000.0, // NEO550 free speed is 11000 RPM
          20, // 20A
          12 // 12V
          );

  /** Drivetrain configuration */
  public static final DrivetrainConfig drivetrainConfig =
      new DrivetrainConfig(
          Units.inchesToMeters(2.84), // wheel diameter
          Units.inchesToMeters(21.5), // width X
          Units.inchesToMeters(21.5), // width Y
          5.0, // max linear speed
          4.0, // max linear acceleration
          driveMotorConfig,
          turnMotorConfig);

  /** Swerve module configurations (FL, FR, RL, RR) */
  public static final ModuleConfig[] moduleConfigs = {
    new ModuleConfig(CanID.MOTOR_DRIVE_FL, CanID.MOTOR_TURN_FL, true),
    new ModuleConfig(CanID.MOTOR_DRIVE_FR, CanID.MOTOR_TURN_FR, true),
    new ModuleConfig(CanID.MOTOR_DRIVE_RL, CanID.MOTOR_TURN_RL, true),
    new ModuleConfig(CanID.MOTOR_DRIVE_RR, CanID.MOTOR_TURN_RR, true),
  };

  /**
   * Swerve module tunable factors (configured defaults).
   *
   * <p>Drive motor FF kV/kA factors are taken from https://reca.lc/drive. Note that kA is
   * irrelevant since we are not using accelation in the FF calculations.
   */
  public static final ModuleTunableFactors moduleFactors =
      switch (Constants.currentMode) {
        case REAL, REPLAY -> new ModuleTunableFactors(
            new MotorTunableFactorsPID(0.3, 0.0, 0.0, 0.0),
            new MotorTunableFactorsPID(2.5, 0.0, 0.0, 0.0),
            new MotorTunableFactorsFF(0.2, 2.6, 0.0));
        default -> new ModuleTunableFactors(
            new MotorTunableFactorsPID(0.0, 0.0, 0.0, 0.0),
            new MotorTunableFactorsPID(0.0, 0.0, 0.0, 0.0),
            new MotorTunableFactorsFF(0.0, 0.0, 0.0));
      };

  /** Angular positions of FL, FR, RL, RR modules with respect to the chassis */
  public static final Rotation2d[] moduleAngularOffsets = {
    Rotation2d.fromRadians(-Math.PI / 2),
    Rotation2d.fromRadians(0),
    Rotation2d.fromRadians(Math.PI),
    Rotation2d.fromRadians(Math.PI / 2),
  };

  /** Lateral translations of FL, FR, RL, RR modules with respect to the chassis */
  public static final Translation2d[] moduleTranslations = {
    new Translation2d(drivetrainConfig.trackWidthY / 2.0, drivetrainConfig.trackWidthX / 2.0),
    new Translation2d(drivetrainConfig.trackWidthY / 2.0, -drivetrainConfig.trackWidthX / 2.0),
    new Translation2d(-drivetrainConfig.trackWidthY / 2.0, drivetrainConfig.trackWidthX / 2.0),
    new Translation2d(-drivetrainConfig.trackWidthY / 2.0, -drivetrainConfig.trackWidthX / 2.0)
  };

  /** Determines whether to use omega FF (second-order kinematics) */
  public static final boolean useSecondOrderKinematics = false;

  /** Drive kinematics */
  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(moduleTranslations);

  /** Configurable parameters of the drivetrain. */
  public record DrivetrainConfig(
      double wheelDiameter,
      double trackWidthX,
      double trackWidthY,
      double maxLinearSpeed,
      double maxLinearAcceleration,
      MotorConfig driveMotor,
      MotorConfig turnMotor) {
    public double driveBaseRadius() {
      return Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
    }

    /** Wheel free speed (meters per second) */
    public double wheelFreeSpeed() {
      return driveEncoderVelocityFactor() * driveMotor.freeSpeedRpm;
    }

    /** Maximum commandable chassis angular speed */
    public double maxAngularSpeed() {
      return maxLinearSpeed / driveBaseRadius();
    }

    /** Maximum commandable chassis angular acceleration */
    public double maxAngularAcceleration() {
      return maxAngularSpeed();
    }

    /** Factor to convert internal encoder rotations into distance */
    public double driveEncoderPositionFactor() {
      return (Math.PI * wheelDiameter) / driveMotor.reductionFactor();
    }

    /** Factor to convert internal encoder rotations into linear velocity */
    public double driveEncoderVelocityFactor() {
      return driveEncoderPositionFactor() / 60.0;
    }

    /** Factor to convert internal encoder rotations to radians */
    public double turnEncoderPositionFactor() {
      return 2 * Math.PI;
    }

    /** Factor to convert internal encoder rotations to angular velocity */
    public double turnEncoderVelocityFactor() {
      return turnEncoderPositionFactor() / 60.0;
    }

    /** Maximum commandable angular velocity for a turn motor */
    public double turnMotorMaxAngularSpeed() {
      return (turnEncoderVelocityFactor() * turnMotor.freeSpeedRpm()) / turnMotor.reductionFactor();
    }
  }

  /** Configurable parameters of a swerve module. */
  public record ModuleConfig(int driveCanId, int turnCanId, boolean turnMotorInverted) {}

  /** Tunable factors of a swerve module. */
  public record ModuleTunableFactors(
      MotorTunableFactorsPID driveMotor,
      MotorTunableFactorsPID turnMotor,
      MotorTunableFactorsFF driveMotorFF) {}

  /** Configurable parameters of a drive or turn motor. */
  public record MotorConfig(
      double reductionFactor, double freeSpeedRpm, int currentLimit, int nominalVoltage) {}

  /** Tunable factors (PID) of a drive or turn motor. */
  public record MotorTunableFactorsPID(double kP, double kI, double kD, double kFF) {}

  /** Tunable factors (FF) of a drive motor. */
  public record MotorTunableFactorsFF(double kS, double kV, double kA) {}
}
