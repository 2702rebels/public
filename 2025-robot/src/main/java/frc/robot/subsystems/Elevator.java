package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTalonFXS;
import frc.robot.lib.PhoenixUtil;
import frc.robot.lib.Tunable;

public class Elevator extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kP = 1.99;
  private static final double __kD = 0;

  private static final double __kG0 = 0;
  private static final double __kG1 = 0.3;
  private static final double __kV = 0.075;
  private static final double __kMaxVelocity = 100; // rps
  private static final double __kMaxAcceleration = 0; // rps/s
  private static final double __kJerk = 0;

  // Expo kV and kA characterize the profile not gains
  // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html#motion-magic-expo
  //
  // max velocity (rps) = min(supply voltage / kV, cruise velocity)
  // max acceleration (rps/s) = supply voltage / kA
  //
  // For example,
  // kV = 0.03 at 12V -> max velocity = 400 rps
  // kA = 0.04 at 12V -> max acceleration = 300 rps/s
  //
  // SMALLER VALUES REPRESENT HIGHER MAX VELOCITY / ACCELERATION
  private static final double __kVExpo = 0.03;
  private static final double __kAExpo = 0.04;

  private static final double kStatorCurrentLimit = 60;
  private static final double kStallSpeed = 100; // RPMs

  private double extraFF = 0;

  // conversion factor between rotation and linear movement
  // 3" wheel diameter, 8:1 reduction ratio
  private static final double conversionFactor = Math.PI * 3 / 8;

  // maximum safe travel
  private static final double maxTravel = 67.5;

  // tolerance (inches)
  private static final double tolerance = 0.5;

  @Logged(name = "Setpoint", importance = Importance.INFO)
  private double setpoint;

  @Logged(name = "Leader", importance = Importance.INFO)
  private final LoggedTalonFXS motorLeader;

  @Logged(name = "Follower", importance = Importance.INFO)
  private final LoggedTalonFXS motorFollower;

  private final Follower followerControl;
  private final MotionMagicExpoVoltage request = new MotionMagicExpoVoltage(0).withEnableFOC(true);

  public Elevator(int leaderCanID, int followerCanID, CANBus canbus) {
    motorLeader =
        new LoggedTalonFXS(
            leaderCanID,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
              config.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
              config.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
              config.ExternalFeedback.ExternalFeedbackSensorSource =
                  ExternalFeedbackSensorSourceValue.Commutation;

              config.Slot0.kP = __kP;
              config.Slot0.kD = __kD;
              config.Slot0.kV = __kV;
              config.MotionMagic.MotionMagicCruiseVelocity = __kMaxVelocity;
              config.MotionMagic.MotionMagicAcceleration = __kMaxAcceleration;
              config.MotionMagic.MotionMagicJerk = __kJerk;
              config.MotionMagic.MotionMagicExpo_kV = __kVExpo;
              config.MotionMagic.MotionMagicExpo_kA = __kAExpo;
            });

    motorFollower =
        new LoggedTalonFXS(
            followerCanID,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
              config.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
              config.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
              config.ExternalFeedback.ExternalFeedbackSensorSource =
                  ExternalFeedbackSensorSourceValue.Commutation;
            });

    // follow leader's output (non-inverted)
    followerControl = new Follower(motorLeader.getDeviceID(), false);

    PhoenixUtil.run(
        "Elevator Follow Leader",
        motorFollower.getDeviceID(),
        () -> motorFollower.setControl(followerControl));

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Elevator", motorLeader);
    Tunable.bindMotionProfile("Elevator/Motion", motorLeader);

    reset();
  }

  public boolean isStalled() {
    return motorLeader.getStatorCurrentAmps() > kStatorCurrentLimit * 0.9
        && Math.abs(motorLeader.getVelocityRPM()) < kStallSpeed;
  }

  @Override
  public void periodic() {
    // elevator's effective weight changes as it extends higher, since
    // more sections need to be supported, this requires different kG
    // values to be applied when elevator is near ground vs. extended
    //
    // kG0 - applied when elevator is near zero
    // kG1 - applied otherwise

    final var gravityFF = getPosition() < 16 ? kG0.get() : kG1.get(); // kG0 below 16"
    motorLeader.setControl(
        request.withPosition(setpoint / conversionFactor).withFeedForward(gravityFF + extraFF));
  }

  public void reset() {
    motorLeader.setPosition(0);
  }

  public void setExtraFF(double value) {
    extraFF = value;
  }

  public void setPosition(double valueInches) {
    setpoint = MathUtil.clamp(valueInches, 0, maxTravel);
  }

  public boolean isAtPosition(double valueInches) {
    return isAtPosition(valueInches, tolerance);
  }

  public boolean isAtPosition(double valueInches, double tolerance) {
    return MathUtil.isNear(valueInches, getPosition(), tolerance);
  }

  public boolean isAbovePosition(double valueInches) {
    return getPosition() > valueInches;
  }

  public boolean isBelowPosition(double valueInches) {
    return getPosition() < valueInches;
  }

  @Logged(name = "Position", importance = Importance.INFO)
  public double getPosition() {
    return motorLeader.getPositionRotations() * conversionFactor;
  }

  @Logged(name = "Velocity", importance = Importance.INFO)
  public double getVelocity() {
    return motorLeader.getVelocityRPM() * conversionFactor / 60;
  }

  // spotless:off
  private Tunable.DoubleValue kG0 = Tunable.value("Elevator/FF/kG0", __kG0);
  private Tunable.DoubleValue kG1 = Tunable.value("Elevator/FF/kG1", __kG1);
  // spotless:on
}
