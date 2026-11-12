package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.AprilTagDetector;
import frc.robot.subsystems.vision.NoteDetector;
import frc.robot.util.DefaultCommand;
import frc.robot.util.Misc;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Commands factory. */
public class CommandsFactory {
  private final Drive drive;
  private final Led led;
  private final Pivot pivot;
  private final Intake intake;
  private final Shooter shooter;
  private final Climb climb;
  private final AprilTagDetector aprilTagDetector;
  private final NoteDetector noteDetector;

  public static final Rotation2d[] circleOrientations =
      Arrays.stream(DriveConstants.moduleTranslations)
          .map(translation -> translation.getAngle().plus(new Rotation2d(Math.PI / 2.0)))
          .toArray(Rotation2d[]::new);

  public CommandsFactory(
      Drive drive,
      Led led,
      Intake intake,
      Pivot pivot,
      Shooter shooter,
      Climb climb,
      AprilTagDetector aprilTagDetector,
      NoteDetector noteDetector) {
    this.drive = drive;
    this.led = led;
    this.intake = intake;
    this.pivot = pivot;
    this.shooter = shooter;
    this.climb = climb;
    this.aprilTagDetector = aprilTagDetector;
    this.noteDetector = noteDetector;
  }

  public static enum LedMode {
    /** Default mode */
    DEFAULT,
    /** Disabled */
    DISABLED,
    /** Post-game logging */
    LOGGING,
    /** Amplifier shooting mode */
    AMP,
    /** Acquired note */
    ACQUIRED_NOTE,
    /** Acquired AprilTags via vision */
    TRACKING_TAGS,

    SPEAKER,
  }

  /** Returns lambda that implements the mode. */
  private Runnable createLedFunction(LedMode mode) {
    switch (mode) {
      case DISABLED:
        return () -> led.random(Misc.toHSV(Color.kRed), 2.5, 10);
      case LOGGING:
        return () -> led.flash(Color.kMagenta, 0.25);
      case AMP:
        return () -> led.breath(Color.kWhite, 3);
      case ACQUIRED_NOTE:
        return () -> led.flash(Color.kOrange, 0.25);
      case TRACKING_TAGS:
        return () -> led.random(Misc.toHSV(Color.kGreen), 2.5, 10);
      case SPEAKER:
        return () -> led.flash(Color.kRed, 0.25);
      default:
        return () -> led.breath(Color.kRed, 3);
    }
  }

  /** Commands the specified LED mode. */
  public Command led(LedMode mode) {
    final var command = new DefaultCommand(createLedFunction(mode), led);
    command.setName("LED_" + mode.toString());
    return command.ignoringDisable(true);
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public Command driveTeleop(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
    return new DriveTeleop(drive, xSupplier, ySupplier, rotationSupplier);
  }

  /** Commands sequence to score in speaker objective. */
  public Command scoreInSpeaker() {
    return new ScoreInSpeaker(drive, shooter, pivot, aprilTagDetector).withName("ScoreInSpeaker");
  }

  /** Commands intake to run until the front beam is tripped. */
  public Command intake() {
    return new FunctionalCommand(
            () -> {
              intake.start();
              pivot.setRequestedAngle(90);
              shooter.feedForward();
              // if (DriverStation.isAutonomous()) {
              shooter.reverseFlywheel();
              // }

              // stop feeding once note is far enough into the shooter
              if (shooter.isFrontBeamTripped()) {
                shooter.stopFeed();
              }
            },
            () -> {},
            interrupted -> {
              intake.stop();
              pivot.setRequestedAngle(20);
              // if (DriverStation.isAutonomous()) {
              shooter.stop();
              // } else {
              //   shooter.stopFeed();
              // }
            },
            shooter::isFrontBeamTrippedAndTimeout,
            intake,
            pivot,
            shooter)
        .withName("Intake");
  }

  /** Commands ejection of the game piece by running the intake in reverse. */
  public Command eject() {
    return new StartEndCommand(intake::reverse, intake::stop, intake).withName("Eject");
  }

  /** Commands pivot arm to position until it arrives there within specified tolerance. */
  public Command moveArmToPosition(double angle, double allowance) {
    return new FunctionalCommand(
            () -> pivot.setRequestedAngle(angle),
            () -> {},
            interupted -> {},
            () -> pivot.isAtGoal(allowance),
            pivot)
        .withName("MoveArmToPosition");
  }

  /** Commands pivot arm to position until it arrives there within specified tolerance. */
  public Command moveArmToPosition(DoubleSupplier angle, double allowance) {
    return new FunctionalCommand(
        () -> pivot.setRequestedAngle(angle.getAsDouble()),
        () -> {},
        interupted -> {},
        () -> pivot.isAtGoal(allowance),
        pivot);
  }

  /** Commands pivot arm to position until it arrives there within the default tolerance. */
  public Command moveArmToPosition(double angle) {
    return new FunctionalCommand(
            () -> pivot.setRequestedAngle(angle),
            () -> {},
            interupted -> {},
            pivot::isAtGoal,
            pivot)
        .withName("MoveArmToPosition");
  }

  /** Commands pivot to disable the control loop. */
  public Command disablePivot() {
    return Commands.runOnce(pivot::disable, pivot).ignoringDisable(true).withName("DisablePivot");
  }

  /**
   * Commands shooter to execute a shot by spinning the flywheel to target RPM and engaging feeder
   * once the flywheel is at speed and ready supplier returns true.
   */
  public Command shoot(double rpm, BooleanSupplier ready) {
    return new FunctionalCommand(
            () -> shooter.setSpeed(rpm),
            () -> {
              if (shooter.isAtSpeed(rpm) && ready.getAsBoolean()) {
                shooter.feedOut();
              }
            },
            (interupted) -> shooter.stop(),
            () -> false,
            shooter)
        .withName("shoot");
  }

  public Command shoot(DoubleSupplier rpm, BooleanSupplier ready) {
    return new FunctionalCommand(
        () -> shooter.setSpeed(rpm.getAsDouble()),
        () -> {
          if (shooter.isAtSpeed(rpm.getAsDouble()) && ready.getAsBoolean()) {
            shooter.feedOut();
          }
        },
        (interupted) -> shooter.stop(),
        () -> false,
        shooter);
  }

  /** Commands shooter to start feeding the game piece until the front beam is tripped. */
  public Command feedIn() {
    return new StartEndCommand(shooter::feedForward, shooter::stopFeed, shooter)
        .until(shooter::isFrontBeamTripped)
        .withName("FeedIn");
  }

  public Command feedInSlow() {
    return new StartEndCommand(() -> shooter.feedForward(0.2), shooter::stopFeed, shooter)
        .until(shooter::isFrontBeamTripped)
        .withName("FeedInSlow");
  }

  /** Commands shooter to reindex the game piece. */
  public Command reindex() {
    return new FunctionalCommand(
            () -> {
              shooter.feedReverse();
              shooter.reverseFlywheel();
            },
            () -> {},
            (interrupted) -> shooter.stop(),
            () -> !shooter.isFrontBeamTripped(),
            shooter)
        .andThen(feedInSlow())
        .withName("Reindex");
  }

  /** Commands hooks to home until the limit swith closes. */
  public Command homeHooks() {
    return new FunctionalCommand(
            () -> {
              climb.releaseRatchet();
              climb.home();
            },
            () -> {},
            interrupted -> {
              climb.setHome();
              climb.stop();
            },
            () -> climb.isLimitSwitchClosed(),
            climb)
        .withName("HomeHooks");
  }

  /** Commands hooks to retract until the position is reached or the limit switch closes. */
  public Command retractHooks() {
    return new FunctionalCommand(
            () -> {
              climb.engageRatchet();
              climb.setRetractPosition();
            },
            () -> {},
            (interrupted) -> climb.stop(),
            () -> climb.isLimitSwitchClosed() || climb.isAtRetractGoal(),
            climb)
        .withName("RetractHooks");
  }

  /** Commands hooks to raise until they reach the top position. */
  public Command raiseHooks() {
    final var relax =
        new FunctionalCommand(
                () -> {
                  climb.releaseRatchet();
                  climb.setRelaxPosition();
                },
                () -> {},
                (interrupted) -> climb.stop(),
                climb::isAtDesiredGoal,
                climb)
            .withName("RelaxClimb");

    final var raise =
        new FunctionalCommand(
                () -> climb.setRaisePosition(),
                () -> {},
                (interrupted) -> climb.stop(),
                climb::isAtRaisedGoal,
                climb)
            .withName("RaiseClimb");

    return Commands.either(relax.andThen(Commands.waitSeconds(0.25)), homeHooks(), climb::isHomed)
        .andThen(raise)
        .withName("RaiseHooks");
  }

  /** Commands climb ratchet to engage. */
  public Command engageRatchet() {
    return new InstantCommand(climb::engageRatchet, climb).withName("EngageRatchet");
  }

  /** Commands climb ratchet to release. */
  public Command releaseRatchet() {
    return new InstantCommand(climb::releaseRatchet, climb).withName("ReleaseRatchet");
  }

  /** Commands wheel diameter characterization routine. */
  public Command wheelDiameterCharacterization() {
    return drive
        .orientModules(circleOrientations)
        .withTimeout(2)
        .andThen(new WheelDiameterCharacterization(drive, 1))
        .withName("WheelDiameterCharacterization");
  }

  /** Resets pose. */
  public Command setPose(Supplier<Pose2d> pose) {
    return Commands.runOnce(() -> drive.setPose(pose.get()), drive);
  }

  /** Commands robot to drive to the supplied pose. */
  public Command driveToPose(
      Supplier<Pose2d> pose, boolean stopAtGoal, BooleanSupplier stopCondition) {
    return new DriveToPose(drive, pose, stopAtGoal, stopCondition);
  }

  public Command driveToPose(Pose2d pose) {
    return new DriveToPose(drive, pose);
  }

  /** Drives to note via vision. */
  public Command driveToNote() {
    return new DriveToNoteVision(drive, noteDetector, intake, shooter);
  }

  /** Commands robot to drive and intake the note via vision. */
  public Command autoIntakeViaVision() {
    return driveToNote().deadlineWith(intake());
  }

  public Command trapShot() {
    return moveArmToPosition(() -> SmartDashboard.getNumber("Trap/Angle", 59), 1)
        .alongWith(shoot(() -> SmartDashboard.getNumber("Trap/RPM", 2200), () -> pivot.isAtGoal()))
        .until(() -> !shooter.hasGamePiece());
  }

  public Command pass() {
    return moveArmToPosition(20)
        .alongWith(shoot(1000, () -> pivot.isAtGoal(3)))
        .until(() -> !shooter.hasGamePiece())
        .andThen(Commands.runOnce(() -> shooter.stop()));
  }
}
