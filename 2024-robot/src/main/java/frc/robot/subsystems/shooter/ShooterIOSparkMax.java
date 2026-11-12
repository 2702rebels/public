package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.CanID;
import frc.robot.Constants;
import frc.robot.DioID;
import frc.robot.util.SparkMaxManager;

public class ShooterIOSparkMax implements ShooterIO {
  private final CANSparkMax flywheelMotor =
      new CANSparkMax(CanID.MOTOR_SHOOTER_LEADER, MotorType.kBrushless);
  private final CANSparkMax followerMotor =
      new CANSparkMax(CanID.MOTOR_SHOOTER_FOLLOWER, MotorType.kBrushless);
  private final CANSparkMax feedMotor =
      new CANSparkMax(CanID.MOTOR_SHOOTER_FEED, MotorType.kBrushless);

  private final RelativeEncoder flywheelEncoder = flywheelMotor.getEncoder();
  private final RelativeEncoder followerEncoder = followerMotor.getEncoder();
  private final RelativeEncoder feedEncoder = feedMotor.getEncoder();
  private final SparkPIDController flywheelPID = flywheelMotor.getPIDController();
  private final SparkPIDController feedPID = feedMotor.getPIDController();

  private final DigitalInput beambreakFront = new DigitalInput(DioID.BEAMBREAK_SHOOTER_FRONT);
  private final DigitalInput beambreakBack = new DigitalInput(DioID.BEAMBREAK_SHOOTER_BACK);

  public ShooterIOSparkMax() {
    SparkMaxManager.initializeAndBurn(flywheelMotor, this::initFlywheel);
    SparkMaxManager.initializeAndBurn(followerMotor, this::initFollower);
    SparkMaxManager.initializeAndBurn(feedMotor, this::initFeed);
  }

  private boolean initFlywheel() {
    // spotless:off
    int errors = 0;

    errors += SparkMaxManager.check(flywheelMotor.setSmartCurrentLimit(60));
    errors += SparkMaxManager.check(flywheelMotor.setIdleMode(IdleMode.kCoast));
    errors += SparkMaxManager.check(flywheelMotor.enableVoltageCompensation(12));

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

    errors += SparkMaxManager.check(flywheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100));
    errors += SparkMaxManager.check(flywheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, framePeriod));
    errors += SparkMaxManager.check(flywheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, framePeriod));
    errors += SparkMaxManager.check(flywheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535));
    errors += SparkMaxManager.check(flywheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535));
    errors += SparkMaxManager.check(flywheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535));
    errors += SparkMaxManager.check(flywheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535));

    // spotless:on
    return errors == 0;
  }

  private boolean initFollower() {
    // spotless:off
    int errors = 0;

    errors += SparkMaxManager.check(followerMotor.follow(flywheelMotor, true));
    errors += SparkMaxManager.check(followerMotor.setSmartCurrentLimit(60));
    errors += SparkMaxManager.check(followerMotor.setIdleMode(IdleMode.kCoast));

    // set this so it can't apply negative power to try and pull the flywheel down to speed
    errors += SparkMaxManager.check(flywheelPID.setOutputRange(0, 1.0));

    final var framePeriod = (int) (1000.0 * Constants.cyclePeriodSeconds);

    errors += SparkMaxManager.check(followerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100));
    errors += SparkMaxManager.check(followerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, framePeriod));
    errors += SparkMaxManager.check(followerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100));
    errors += SparkMaxManager.check(followerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535));
    errors += SparkMaxManager.check(followerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535));
    errors += SparkMaxManager.check(followerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535));
    errors += SparkMaxManager.check(followerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535));

    // spotless:on
    return errors == 0;
  }

  private boolean initFeed() {
    // spotless:off
    int errors = 0;

    feedMotor.setInverted(true);
    errors += SparkMaxManager.check(feedMotor.setIdleMode(IdleMode.kBrake));
    errors += SparkMaxManager.check(feedPID.setOutputRange(0, 1.0));

    final var framePeriod = (int) (1000.0 * Constants.cyclePeriodSeconds);

    errors += SparkMaxManager.check(feedMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100));
    errors += SparkMaxManager.check(feedMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, framePeriod));
    errors += SparkMaxManager.check(feedMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, framePeriod));
    errors += SparkMaxManager.check(feedMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535));
    errors += SparkMaxManager.check(feedMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535));
    errors += SparkMaxManager.check(feedMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535));
    errors += SparkMaxManager.check(feedMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535));

    // spotless:on
    return errors == 0;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.flywheelVelocity = flywheelEncoder.getVelocity();
    inputs.flywheelPosition = flywheelEncoder.getPosition();
    inputs.flywheelAppliedVolts = flywheelMotor.getAppliedOutput() * flywheelMotor.getBusVoltage();
    inputs.flywheelCurrentAmps = flywheelMotor.getOutputCurrent();

    inputs.followerVelocity = followerEncoder.getVelocity();
    inputs.followerAppliedVolts = followerMotor.getAppliedOutput() * followerMotor.getBusVoltage();
    inputs.followerCurrentAmps = followerMotor.getOutputCurrent();

    inputs.feedVelocity = feedEncoder.getVelocity();
    inputs.feedPosition = feedEncoder.getPosition();
    inputs.feedAppliedVolts = feedMotor.getAppliedOutput() * feedMotor.getBusVoltage();
    inputs.feedCurrentAmps = feedMotor.getOutputCurrent();

    inputs.beamBreakFront = !beambreakFront.get();
    inputs.beamBreakBack = !beambreakBack.get();
  }

  @Override
  public void configureFlywheelPID(double kP, double kI, double kD, double kff, double staticGain) {
    flywheelPID.setP(kP);
    flywheelPID.setI(kI);
    flywheelPID.setD(kD);
    flywheelPID.setFF(kff);
  }

  @Override
  public void setFlywheelSpeed(double rpm) {
    final var ff = 0.2265; // + rpm * 0.0022
    flywheelPID.setReference(rpm, ControlType.kVelocity, 0, ff);
  }

  @Override
  public void setFlywheelPercentOutput(double percent) {
    flywheelMotor.set(percent);
  }

  @Override
  public void stopFlywheel() {
    flywheelMotor.stopMotor();
  }

  @Override
  public void configureFeedPID(double kP, double kI, double kD, double kFF) {
    feedPID.setP(kP);
    feedPID.setI(kI);
    feedPID.setD(kD);
    feedPID.setFF(kFF);
  }

  @Override
  public void setFeedSpeed(double rpm) {
    feedPID.setReference(rpm, ControlType.kVelocity);
  }

  @Override
  public void setFeedPercentOutput(double percent) {
    feedMotor.set(percent);
  }

  @Override
  public void stopFeed() {
    feedMotor.stopMotor();
  }

  @Override
  public void setFlywheelBrakeMode(boolean enable) {
    flywheelMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    followerMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setFeedBrakeMode(boolean enable) {
    feedMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setVoltage(double volts) {
    flywheelMotor.setVoltage(volts);
  }
}
