package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.LoggedSparkMax;
import frc.robot.lib.RevUtil;
import frc.robot.lib.Tunable;

/**
 * Controls motors responsible for manipulating game pieces (coral, algae), enables intake/eject
 * motion of the game piece.
 */
public final class Manipulator extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  private static final double __kCoralIntakeVelocity = 0.85;
  private static final double __kCoralIntakeVelocityGuide = 1;
  private static final double __kCoralEjectVelocity = 0.4;
  private static final double __kCoralSlowEjectVelocity = 0.2;
  private static final double __kCoralReverseTime = 0.06;
  private static final double __kCoralStallSpeedUB = 800; // RPMs
  private static final double __kCoralStallSpeedLB = 400; // RPMs

  private static final double __kAlgaeIntakeVelocity = 1.0;
  private static final double __kAlgaeEjectVelocity = 1.0;
  private static final double __kAlgaeHoldVelocity = 1.0;

  private static final double __kCoralP = 3.5;
  private static final double __kCoralReindexPosition = 0.15;

  private static final int __kGuideMotorStallCurrentAlgae = 20;
  private static final int __kGuideMotorStallCurrentCoral = 30;

  private static final double kGuideGearRatio = 7;
  private static final double kCoralGearRatio = 7;

  private double motorGuideSpeed = 0;

  private boolean intakingCoral = false;
  private boolean ejectingAlgae = false;

  private boolean reversedGuide = false;
  private boolean stallDetectionActive = false;

  private long reversedAt = -1;

  private SparkClosedLoopController controllerCoral;

  @Logged(name = "Stalled", importance = Importance.DEBUG)
  public boolean isCoralMotorStalled() {
    // only detect stalling when intaking coral and
    // once we have reached target RPMs on the intake
    if (intakingCoral && stallDetectionActive) {
      final var rpm = Math.abs(motorCoral.getVelocityRPM());
      return Math.abs(motorCoral.getAppliedOutput()) > kCoralIntakeVelocity.get() - 0.05
          && rpm > kCoralStallSpeedLB.get()
          && rpm < kCoralStallSpeedUB.get();
    }
    return false;
  }

  @Logged(name = "IsEjectingAlgae", importance = Importance.INFO)
  public Trigger isEjectingAlgae = new Trigger(() -> ejectingAlgae);

  /** Detects algae presence by checking for stall on the guide motor. */
  public boolean hasAlgae() {
    return !intakingCoral
        && motorGuide.getCurrentAmps() > kGuideMotorStallCurrentAlgae.get() * 0.5
        && MathUtil.isNear(0, motorGuide.getVelocityRPM(), 50);
  }

  @Logged(name = "Guide", importance = Importance.INFO)
  private final LoggedSparkMax motorGuide;

  @Logged(name = "Coral", importance = Importance.INFO)
  private final LoggedSparkMax motorCoral;

  public Manipulator(int guideCanID, int coralCanID) {
    motorGuide =
        new LoggedSparkMax(
            guideCanID,
            (config) -> {
              config
                  .smartCurrentLimit(__kGuideMotorStallCurrentCoral)
                  .voltageCompensation(12)
                  .idleMode(IdleMode.kCoast)
                  .signals
                  .primaryEncoderVelocityAlwaysOn(true);

              // conversion factor based on the gear ratio
              config
                  .encoder
                  .positionConversionFactor(1 / kGuideGearRatio)
                  .velocityConversionFactor(1 / kGuideGearRatio);
            });

    motorCoral =
        new LoggedSparkMax(
            coralCanID,
            (config) -> {
              config
                  .smartCurrentLimit(40)
                  .voltageCompensation(12)
                  .idleMode(IdleMode.kCoast)
                  .signals
                  .primaryEncoderVelocityAlwaysOn(true);

              config
                  .closedLoop
                  .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                  .velocityFF(0)
                  .p(__kCoralP);

              // conversion factor based on the gear ratio
              config
                  .encoder
                  .positionConversionFactor(1 / kCoralGearRatio)
                  .velocityConversionFactor(1 / kCoralGearRatio);
            });

    controllerCoral = motorCoral.getClosedLoopController();

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Manipulator/Coral", motorCoral, ClosedLoopSlot.kSlot0);
  }

  private void stopMotors() {
    intakingCoral = false;
    ejectingAlgae = false;
    reversedGuide = false;
    stallDetectionActive = false;
    reversedAt = -1;
    motorGuideSpeed = 0;

    motorCoral.stopMotor();
    motorGuide.stopMotor();
  }

  @Override
  public void periodic() {
    if (intakingCoral) {
      final var timestamp = RobotController.getFPGATime();

      // stall detection only needs to kick in if we have seen coral motor
      // at speeds above the stall upper bound
      if (!stallDetectionActive
          && Math.abs(motorCoral.getVelocityRPM()) > kCoralStallSpeedUB.get()) {
        stallDetectionActive = true;
      }

      // if coral motor is stalling, reverse guide for a brief time
      if (isCoralMotorStalled() && reversedGuide == false) {
        reversedAt = timestamp;
        reversedGuide = true;
      } else if (reversedAt < 0 || timestamp > reversedAt + kCoralReverseTime.get() * 1000000) {
        reversedAt = -1;
        reversedGuide = false;
      }

      // do not reverse if we think the coral is in
      if (getCoralPresenceSensor().isPressed()) {
        reversedGuide = false;
      }

      motorGuide.set((reversedGuide ? -1 : 1) * motorGuideSpeed);
      motorCoral.set(-kCoralIntakeVelocity.get());
    }
  }

  /** Updates smart current limit on the guide motor. */
  private void setGuideStallCurrent(double value) {
    final var newConfig = new SparkMaxConfig();
    newConfig.smartCurrentLimit((int) value);
    RevUtil.configEphemeral(motorGuide, newConfig);
  }

  /**
   * Gets the coral presence sensor.
   *
   * <p>Limit switch is connected to the data port on the SparkMax controller.
   */
  public SparkLimitSwitch getCoralPresenceSensor() {
    return motorGuide.getForwardLimitSwitch();
  }

  @Logged(name = "CoralPosition", importance = Importance.INFO)
  public double getCoralPosition() {
    return motorCoral.getRelativeEncoder().getPosition();
  }

  public Command intakeCoral() {
    return new FunctionalCommand(
            () -> {
              intakingCoral = true;
              stallDetectionActive = false;
              motorGuideSpeed = -kCoralIntakeVelocityGuide.get();
              setGuideStallCurrent(kGuideMotorStallCurrentCoral.get());
            },
            () -> {},
            interrupted -> stopMotors(),
            () -> false,
            this)
        .withName("IntakeCoral");
  }

  public Command ejectCoral() {
    return runEnd(
            () -> {
              motorCoral.set(kCoralEjectVelocity.get());
              motorGuide.stopMotor();
            },
            this::stopMotors)
        .withName("EjectCoral");
  }

  public Command ejectCoralSlow() {
    return runEnd(
            () -> {
              motorCoral.set(kCoralSlowEjectVelocity.get());
              motorGuide.stopMotor();
            },
            this::stopMotors)
        .withName("EjectCoralSlow");
  }

  public Command ejectCoralReverse() {
    return runEnd(
            () -> {
              motorCoral.set(-kCoralEjectVelocity.get());
              motorGuide.stopMotor();
            },
            this::stopMotors)
        .withName("EjectCoralReverse");
  }

  public Command intakeAlgae() {
    return runEnd(
            () -> {
              intakingCoral = false;
              motorCoral.stopMotor();
              motorGuide.set(kAlgaeIntakeVelocity.get());
              setGuideStallCurrent(kGuideMotorStallCurrentAlgae.get());
            },
            this::stopMotors)
        .withName("IntakeAlgae");
  }

  public Command ejectAlgae() {
    return runEnd(
            () -> {
              ejectingAlgae = true;
              motorCoral.stopMotor();
              motorGuide.set(-kAlgaeEjectVelocity.get());
            },
            this::stopMotors)
        .withName("EjectAlgae");
  }

  public Command holdAlgae() {
    return runEnd(
            () -> {
              intakingCoral = false;
              motorCoral.stopMotor();
              motorGuide.set(kAlgaeHoldVelocity.get());
              setGuideStallCurrent(kGuideMotorStallCurrentAlgae.get());
            },
            this::stopMotors)
        .withName("HoldAlgae");
  }

  public Command reindexCoral() {
    return startRun(
            () -> {
              motorCoral.getRelativeEncoder().setPosition(0);
              motorGuide.stopMotor();
            },
            () -> {
              controllerCoral.setReference(kCoralReindexPosition.get(), ControlType.kPosition);
            })
        .withName("ReindexCoral");
  }

  public Command stop() {
    return run(this::stopMotors).withName("ManipulatorStop");
  }

  // spotless:off
  private static Tunable.DoubleValue kCoralIntakeVelocity = Tunable.value("Manipulator/Coral/IntakeVelocity", __kCoralIntakeVelocity);
  private static Tunable.DoubleValue kCoralIntakeVelocityGuide = Tunable.value("Manipulator/Coral/IntakeVelocityGuide", __kCoralIntakeVelocityGuide);
  private static Tunable.DoubleValue kCoralEjectVelocity = Tunable.value("Manipulator/Coral/EjectVelocity", __kCoralEjectVelocity);
  private static Tunable.DoubleValue kCoralSlowEjectVelocity = Tunable.value("Manipulator/Coral/SlowEjectVelocity", __kCoralSlowEjectVelocity);
  private static Tunable.DoubleValue kCoralStallSpeedUB = Tunable.value("Manipulator/Coral/StallSpeedUB", __kCoralStallSpeedUB);
  private static Tunable.DoubleValue kCoralStallSpeedLB = Tunable.value("Manipulator/Coral/StallSpeedLB", __kCoralStallSpeedLB);
  private static Tunable.DoubleValue kCoralReverseTime = Tunable.value("Manipulator/Coral/ReverseTime", __kCoralReverseTime);

  private static Tunable.DoubleValue kAlgaeIntakeVelocity = Tunable.value("Manipulator/Algae/IntakeVelocity", __kAlgaeIntakeVelocity);
  private static Tunable.DoubleValue kAlgaeEjectVelocity = Tunable.value("Manipulator/Algae/EjectVelocity", __kAlgaeEjectVelocity);
  private static Tunable.DoubleValue kAlgaeHoldVelocity = Tunable.value("Manipulator/Algae/HoldVelocity", __kAlgaeHoldVelocity);

  private static Tunable.DoubleValue kCoralReindexPosition = Tunable.value("Manipulator/Coral/ReindexPosition", __kCoralReindexPosition);

  private static Tunable.DoubleValue kGuideMotorStallCurrentAlgae = Tunable.value("Manipulator/Algae/StallCurrent", __kGuideMotorStallCurrentAlgae);
  private static Tunable.DoubleValue kGuideMotorStallCurrentCoral = Tunable.value("Manipulator/Coral/StallCurrent", __kGuideMotorStallCurrentCoral);
  // spotless:on
}
