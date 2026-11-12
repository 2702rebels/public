package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import frc.robot.CanID;
import frc.robot.Constants;
import frc.robot.util.SparkMaxManager;

public class PivotIOSparkMax implements PivotIO {
  private final CANSparkMax motor = new CANSparkMax(CanID.MOTOR_PIVOT, MotorType.kBrushless);
  private final SparkAbsoluteEncoder absolutEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  private final SparkPIDController pid = motor.getPIDController();

  public PivotIOSparkMax() {
    SparkMaxManager.initializeAndBurn(motor, this::init);
  }

  private boolean init() {
    // spotless:off
    int errors = 0;

    // configure controller
    errors += SparkMaxManager.check(motor.setIdleMode(IdleMode.kBrake));
    errors += SparkMaxManager.check(motor.setSmartCurrentLimit(20));
    motor.setInverted(true);

    // configure encoder
    // errors += SparkMaxManager.check(absolutEncoder.setAverageDepth(2));
    errors += SparkMaxManager.check(absolutEncoder.setPositionConversionFactor(2 * Math.PI));
    errors += SparkMaxManager.check(absolutEncoder.setVelocityConversionFactor((2 * Math.PI) / 60));
    errors += SparkMaxManager.check(absolutEncoder.setInverted(true));

    // configure PID
    errors += SparkMaxManager.check(pid.setFeedbackDevice(absolutEncoder));

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

    final var framePeriod = (int) (1000.0 * Constants.cyclePeriodSeconds);

    errors += SparkMaxManager.check(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100));
    errors += SparkMaxManager.check(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100));
    errors += SparkMaxManager.check(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100));
    errors += SparkMaxManager.check(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535));
    errors += SparkMaxManager.check(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535));
    errors += SparkMaxManager.check(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, framePeriod));
    errors += SparkMaxManager.check(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, framePeriod));

    // spotless:on
    return errors == 0;
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.measuredPositionRadians = absolutEncoder.getPosition();
    inputs.measuredVelocityRadiansPerSecond = absolutEncoder.getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    motor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void configureJointPID(double kP, double kI, double kD) {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
  }

  @Override
  public void setJointAngle(double angleRadians) {
    pid.setReference(angleRadians, ControlType.kPosition, 0);
  }

  @Override
  public void setJointAngle(double angleRadians, double ff) {
    pid.setReference(angleRadians, ControlType.kPosition, 0, ff);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
