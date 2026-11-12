package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.CanID;
import frc.robot.Constants;
import frc.robot.DioID;
import frc.robot.util.SparkMaxManager;

public class ClimbIOSparkMax implements ClimbIO {
  private static final int CLIMB_SERVO = 0;
  private CANSparkMax motor = new CANSparkMax(CanID.MOTOR_CLIMB, MotorType.kBrushless);
  private RelativeEncoder encoder = motor.getEncoder();
  private SparkPIDController pid = motor.getPIDController();
  private Servo ratchet = new Servo(CLIMB_SERVO);

  private DigitalInput limitSwitchClosed = new DigitalInput(DioID.LIMITSWITCH_CLIMB);

  public ClimbIOSparkMax() {
    SparkMaxManager.initializeAndBurn(motor, this::init);
  }

  private boolean init() {
    // spotless:off
    int errors = 0;

    errors += SparkMaxManager.check(motor.setSmartCurrentLimit(60));
    errors += SparkMaxManager.check(motor.setIdleMode(IdleMode.kBrake));

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
    errors += SparkMaxManager.check(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, framePeriod));
    errors += SparkMaxManager.check(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535));
    errors += SparkMaxManager.check(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535));
    errors += SparkMaxManager.check(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535));
    errors += SparkMaxManager.check(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535));

    // spotless:on
    return errors == 0;
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.limitSwitchClosed = limitSwitchClosed.get();
  }

  @Override
  public void zeroEncoder() {
    encoder.setPosition(0);
  }

  @Override
  public void setPercentOutput(double speed) {
    motor.set(speed);
  }

  @Override
  public void setPosition(double position) {
    pid.setReference(position, ControlType.kPosition);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
  }

  public void setServoPosition(double degrees) {
    ratchet.setAngle(degrees);
  }
}
