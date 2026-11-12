package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.CanID;
import frc.robot.Constants;
import frc.robot.DioID;
import frc.robot.util.SparkMaxManager;

public class IntakeIOSparkMax implements IntakeIO {
  private final CANSparkMax motor = new CANSparkMax(CanID.MOTOR_INTAKE, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final DigitalInput beambreak = new DigitalInput(DioID.BEAMBREAK_INTAKE);

  public IntakeIOSparkMax() {
    SparkMaxManager.initializeAndBurn(motor, this::init);
  }

  private boolean init() {
    // spotless:off
    int errors = 0;

    errors += SparkMaxManager.check(motor.setSmartCurrentLimit(60));
    errors += SparkMaxManager.check(motor.setIdleMode(IdleMode.kCoast));
    motor.setInverted(false);

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
    errors += SparkMaxManager.check(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, framePeriod));
    errors += SparkMaxManager.check(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100));
    errors += SparkMaxManager.check(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535));
    errors += SparkMaxManager.check(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535));
    errors += SparkMaxManager.check(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535));
    errors += SparkMaxManager.check(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535));

    // spotless:on
    return errors == 0;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocityRPM = encoder.getVelocity();
    inputs.temperature = motor.getMotorTemperature();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.beamBreak = !beambreak.get();
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
