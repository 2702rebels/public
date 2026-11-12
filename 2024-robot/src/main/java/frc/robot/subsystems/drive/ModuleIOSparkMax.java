package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveConstants.DrivetrainConfig;
import frc.robot.subsystems.drive.DriveConstants.ModuleConfig;
import frc.robot.util.SparkMaxManager;
import java.util.OptionalDouble;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnAbsoluteEncoder;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final Rotation2d angularOffset;

  private final SparkPIDController drivePIDController;
  private final SparkPIDController turnPIDController;

  /**
   * Guards values read from the controller against abnormal figures when status frame has not been
   * received due to timeouts or other CAN errors.
   */
  private static OptionalDouble guardRawValue(CANSparkMax controller, double value) {
    return controller.getLastError() == REVLibError.kOk
        ? OptionalDouble.of(value)
        : OptionalDouble.empty();
  }

  public ModuleIOSparkMax(
      ModuleConfig module, Rotation2d angularOffset, DrivetrainConfig drivetrain) {
    this.angularOffset = angularOffset;

    driveMotor = new CANSparkMax(module.driveCanId(), MotorType.kBrushless);
    turnMotor = new CANSparkMax(module.turnCanId(), MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turnAbsoluteEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

    drivePIDController = driveMotor.getPIDController();
    turnPIDController = turnMotor.getPIDController();

    SparkMaxManager.initializeAndBurn(driveMotor, () -> initDrive(module, drivetrain));
    SparkMaxManager.initializeAndBurn(turnMotor, () -> initTurn(module, drivetrain));

    drivePositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(() -> guardRawValue(driveMotor, driveEncoder.getPosition()));

    turnPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(() -> guardRawValue(turnMotor, turnAbsoluteEncoder.getPosition()));
  }

  private boolean initDrive(ModuleConfig module, DrivetrainConfig drivetrain) {
    // spotless:off
    int errors = 0;

    // apply position and velocity conversion factors for the driving encoder;
    // the native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs
    errors += SparkMaxManager.check(driveEncoder.setPositionConversionFactor(drivetrain.driveEncoderPositionFactor()));
    errors += SparkMaxManager.check(driveEncoder.setVelocityConversionFactor(drivetrain.driveEncoderVelocityFactor()));

    // these parameters control the behavior of the velocity filter on the SPARK MAX
    // default settings result in about 100+ms delay due to multiple samples being
    // taken to filter out noise in the velocity readings; reducing depth results
    // in higher noise in data, but reduces the lag
    errors += SparkMaxManager.check(driveEncoder.setMeasurementPeriod(DriveConstants.driveEncoderMeasurementPeriod));
    errors += SparkMaxManager.check(driveEncoder.setAverageDepth(DriveConstants.driveEncoderMeasurementDepth));

    errors += SparkMaxManager.check(drivePIDController.setOutputRange(-1, 1));

    errors += SparkMaxManager.check(driveMotor.setSmartCurrentLimit(drivetrain.driveMotor().currentLimit()));
    errors += SparkMaxManager.check(driveMotor.enableVoltageCompensation(drivetrain.driveMotor().nominalVoltage()));

    errors += SparkMaxManager.check(driveEncoder.setPosition(0.0));

    // tune CAN bus utilization, see:
    // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
    //
    // status 0 - AppliedXyzOutput, Faults, Sticky Faults, Is Follower
    // status 1 - Motor Velocity, Temperature, Voltage, Current
    // status 2 - Motor Position
    // status 3 - Analog Sensor Voltage / Velocity / Position
    // status 4 - Alternate Encoder Velocity / Position
    // status 5 - Duty Cycle Absolute Encoder Position / Absolute Angle
    // status 6 - Duty Cycle Absolute Encoder Velocity / Frequency

    final var odometryFramePeriod = (int) (1000.0 / Constants.odometryFrequency);
    final var velocityFramePeriod = (int) (1000.0 * Constants.cyclePeriodSeconds);

    errors += SparkMaxManager.check(driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100));
    errors += SparkMaxManager.check(driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, velocityFramePeriod));
    errors += SparkMaxManager.check(driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, odometryFramePeriod));
    errors += SparkMaxManager.check(driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535));
    errors += SparkMaxManager.check(driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535));
    errors += SparkMaxManager.check(driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535));
    errors += SparkMaxManager.check(driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535));

    // spotless:on
    return errors == 0;
  }

  private boolean initTurn(ModuleConfig module, DrivetrainConfig drivetrain) {
    // spotless:off
    int errors = 0;

    // apply position and velocity conversion factors for the turning encoder;
    // we want these in radians and radians per second to use with WPILib's swerve APIs
    errors += SparkMaxManager.check(turnAbsoluteEncoder.setPositionConversionFactor(drivetrain.turnEncoderPositionFactor()));
    errors += SparkMaxManager.check(turnAbsoluteEncoder.setVelocityConversionFactor(drivetrain.turnEncoderVelocityFactor()));

    errors += SparkMaxManager.check(turnAbsoluteEncoder.setAverageDepth(DriveConstants.turnEncoderMeasurementDepth));

    // invert the turning encoder, since the output shaft rotates in the opposite
    // direction of the steering motor in the MAXSwerve Module
    errors += SparkMaxManager.check(turnAbsoluteEncoder.setInverted(module.turnMotorInverted()));

    // enable PID wrap around for the turning motor; this will allow the PID controller
    // to go through 0 to get to the setpoint i.e. going from 350 degrees to 10 degrees
    // will go through 0 rather than the other direction which is a longer route
    errors += SparkMaxManager.check(turnPIDController.setPositionPIDWrappingEnabled(true));
    errors += SparkMaxManager.check(turnPIDController.setPositionPIDWrappingMinInput(0));
    errors += SparkMaxManager.check(turnPIDController.setPositionPIDWrappingMaxInput(drivetrain.turnEncoderPositionFactor()));

    // set the PID Controller to use the duty cycle encoder on the swerve
    // module instead of the built in NEO550 encoder
    errors += SparkMaxManager.check(turnPIDController.setFeedbackDevice(turnAbsoluteEncoder));
    errors += SparkMaxManager.check(turnPIDController.setOutputRange(-1, 1));

    errors += SparkMaxManager.check(turnMotor.setSmartCurrentLimit(drivetrain.turnMotor().currentLimit()));
    errors += SparkMaxManager.check(turnMotor.enableVoltageCompensation(drivetrain.turnMotor().nominalVoltage()));

    // tune CAN bus utilization, see:
    // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
    //
    // status 0 - AppliedXyzOutput, Faults, Sticky Faults, Is Follower
    // status 1 - Motor Velocity, Temperature, Voltage, Current
    // status 2 - Motor Position
    // status 3 - Analog Sensor Voltage / Velocity / Position
    // status 4 - Alternate Encoder Velocity / Position
    // status 5 - Duty Cycle Absolute Encoder Position / Absolute Angle
    // status 6 - Duty Cycle Absolute Encoder Velocity / Frequency

    final var odometryFramePeriod = (int) (1000.0 / Constants.odometryFrequency);
    final var velocityFramePeriod = (int) (1000.0 * Constants.cyclePeriodSeconds);

    errors += SparkMaxManager.check(turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100));
    errors += SparkMaxManager.check(turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100));
    errors += SparkMaxManager.check(turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100));
    errors += SparkMaxManager.check(turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535));
    errors += SparkMaxManager.check(turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535));
    errors += SparkMaxManager.check(turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, odometryFramePeriod));
    errors += SparkMaxManager.check(turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, velocityFramePeriod));

    // spotless:on
    return errors == 0;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionMeters = driveEncoder.getPosition();
    inputs.driveVelocityMetersPerSec = driveEncoder.getVelocity();
    inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
    inputs.driveCurrentAmps = driveMotor.getOutputCurrent();

    inputs.turnPosition =
        Rotation2d.fromRadians(turnAbsoluteEncoder.getPosition() - angularOffset.getRadians());
    inputs.turnVelocityRadPerSec = turnAbsoluteEncoder.getVelocity();
    inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
    inputs.turnCurrentAmps = turnMotor.getOutputCurrent();

    inputs.odometryDrivePositionsMeters =
        drivePositionQueue.stream().mapToDouble(Double::valueOf).toArray();

    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRadians(value - angularOffset.getRadians()))
            .toArray(Rotation2d[]::new);

    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnMotor.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void configureDrivePID(double kP, double kI, double kD, double kFF) {
    drivePIDController.setP(kP);
    drivePIDController.setI(kI);
    drivePIDController.setD(kD);
    drivePIDController.setFF(kFF);
  }

  @Override
  public void configureTurnPID(double kP, double kI, double kD, double kFF) {
    turnPIDController.setP(kP);
    turnPIDController.setI(kI);
    turnPIDController.setD(kD);
    turnPIDController.setFF(kFF);
  }

  @Override
  public void setDriveVelocity(double speedMetersPerSecond) {
    drivePIDController.setReference(speedMetersPerSecond, ControlType.kVelocity);
  }

  @Override
  public void setDriveVelocity(double speedMetersPerSecond, double ff) {
    drivePIDController.setReference(speedMetersPerSecond, ControlType.kVelocity, 0, ff);
  }

  @Override
  public void setTurnAngle(double angleRadians) {
    turnPIDController.setReference(angleRadians, ControlType.kPosition);
  }

  @Override
  public void setTurnAngle(double angleRadians, double ff) {
    turnPIDController.setReference(angleRadians, ControlType.kPosition, 0, ff);
  }

  @Override
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }
}
