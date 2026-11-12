package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.LoggedSparkMax;
import frc.robot.lib.Tunable;

public final class Climb extends SubsystemBase {
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kP = 0.15;
  private static final double __kD = 0;

  private static final double __kHoldingSpeed = 0.03;
  private static final double __kServoEngaged = 10;
  private static final double __kServoReleased = 120;
  private static final double __kReadyPosition = 14.5;
  private static final double __kEngagedPosition = 0;

  private static final double conversionFactor =
      Math.PI * 3 / 50; // 3" diameter, 50:1 reduction ratio

  @Logged(name = "Winch", importance = Importance.INFO)
  private final LoggedSparkMax motor;

  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final Servo ratchet;
  private final DigitalInput limitSwitch;
  private boolean holding;
  private boolean locked;

  // tolerance (inches)
  private static final double tolerance = 0.5;

  @Logged(name = "Setpoint", importance = Importance.INFO)
  private double setpoint;

  public Climb(int canID, int switchID, int servoID) {
    motor =
        new LoggedSparkMax(
            canID,
            (config) -> {
              config
                  .smartCurrentLimit(60)
                  .voltageCompensation(12)
                  .idleMode(IdleMode.kBrake)
                  .inverted(true);

              config.signals.primaryEncoderPositionAlwaysOn(true);

              config
                  .closedLoop
                  .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                  .velocityFF(0)
                  .p(__kP)
                  .d(__kD);

              config.encoder.positionConversionFactor(conversionFactor);
            });

    encoder = motor.getEncoder();
    controller = motor.getClosedLoopController();

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Climb", motor, ClosedLoopSlot.kSlot0);

    limitSwitch = new DigitalInput(switchID);
    ratchet = new Servo(servoID);
    isLocked = new Trigger(() -> locked || !limitSwitch.get());

    holding = true;
    resetPosition();
    releaseRatchet();
  }

  @Override
  public void periodic() {
    if (holding) {
      hold();
    } else {
      // set locked mode once we see switch hit
      if (!limitSwitch.get()) {
        locked = true;
      }

      controller.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
  }

  private void engageRatchet() {
    ratchet.setAngle(servoEngaged.get());
  }

  private void releaseRatchet() {
    ratchet.setAngle(servoReleased.get());
  }

  private void resetPosition() {
    encoder.setPosition(0);
  }

  private void hold() {
    motor.set(holdingSpeed.get());
  }

  public void setPosition(double value) {
    setpoint = value;
  }

  public boolean isAtPosition(double value) {
    return isAtPosition(value, tolerance);
  }

  public boolean isAtPosition(double value, double tolerance) {
    return MathUtil.isNear(value, getPosition(), tolerance);
  }

  @Logged(name = "Position", importance = Importance.INFO)
  public double getPosition() {
    return encoder.getPosition();
  }

  @Logged(name = "IsLocked", importance = Importance.INFO)
  public Trigger isLocked;

  @Logged(name = "IsHolding", importance = Importance.INFO)
  public Trigger isHolding = new Trigger(() -> holding);

  public Command reset() {
    return run(() -> {
          setPosition(0);
        })
        .until(() -> isAtPosition(0, 1))
        .andThen(
            runOnce(
                () -> {
                  releaseRatchet();
                  holding = true;
                }));
  }

  public Command ready() {
    return runOnce(
            () -> {
              holding = false;
              releaseRatchet();
              resetPosition();
            })
        .andThen(Commands.waitSeconds(0.2))
        .andThen(runOnce(() -> setPosition(-readyPosition.get())));
  }

  public Command engage() {
    return runOnce(
        () -> {
          holding = false;
          engageRatchet();
          setPosition(-engagedPosition.get());
        });
  }

  public Command setLocked() {
    return runOnce(
        () -> {
          locked = true;
        });
  }

  // spotless:off
  private static Tunable.DoubleValue holdingSpeed = Tunable.value("Climb/HoldingSpeed", __kHoldingSpeed);
  private static Tunable.DoubleValue servoReleased = Tunable.value("Climb/ServoReleased", __kServoReleased);
  private static Tunable.DoubleValue servoEngaged = Tunable.value("Climb/ServoEngaged", __kServoEngaged);
  private static Tunable.DoubleValue readyPosition = Tunable.value("Climb/ReadyPosition", __kReadyPosition);
  private static Tunable.DoubleValue engagedPosition = Tunable.value("Climb/EngagedPosition", __kEngagedPosition);
  // spotless:on
}
