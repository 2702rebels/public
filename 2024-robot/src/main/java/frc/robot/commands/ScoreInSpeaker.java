package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.AprilTag;
import frc.robot.subsystems.vision.AprilTagDetector;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

public class ScoreInSpeaker extends Command {
  // spotless:off
  private static final LoggedDashboardBoolean disableOdometryHistory = new LoggedDashboardBoolean("Score/DisableOdometryHistory", false);
  private static final LoggedTunableNumber virtualTagToleranceAuto = new LoggedTunableNumber("Score/VTag Tolerance Auto", 0.5);
  private static final LoggedTunableNumber virtualTagToleranceTeleop = new LoggedTunableNumber("Score/VTag Tolerance Teleop", 0.2);
  // spotless:on

  private final Drive drive;
  private final Shooter shooter;
  private final Pivot pivot;
  private final AprilTagDetector aprilTagDetector;
  private final RobotState state;
  private AprilTag tag;
  private AprilTag virtualTag;
  private boolean goodToShoot = false;
  private double lastSeenVirtualTagAt;
  private double virtualTagTolerance;

  public ScoreInSpeaker(
      Drive drive, Shooter shooter, Pivot pivot, AprilTagDetector aprilTagDetector) {
    this.drive = drive;
    this.shooter = shooter;
    this.pivot = pivot;
    this.aprilTagDetector = aprilTagDetector;
    this.state = RobotState.getInstance();

    addRequirements(shooter, pivot);
  }

  @Override
  public void initialize() {
    // get the tag we need to be scoring on
    tag = aprilTagDetector.getTag(DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7);
    virtualTag =
        aprilTagDetector.getTag(DriverStation.getAlliance().get() == Alliance.Red ? 17 : 18);
    goodToShoot = false;

    virtualTagTolerance =
        DriverStation.isAutonomous()
            ? virtualTagToleranceAuto.get()
            : virtualTagToleranceTeleop.get();
  }

  @Override
  public void execute() {
    final var timestamp = Timer.getFPGATimestamp();

    var preferredTag =
        timestamp - lastSeenVirtualTagAt < virtualTagTolerance
            ? virtualTag
            : virtualTag.tracked ? virtualTag : tag;

    if (virtualTag.tracked) {
      lastSeenVirtualTagAt = timestamp;
    }

    var z = preferredTag.z;
    var x = preferredTag.x;

    final var epoch = RobotState.getInstance().autoStartedAt;
    final var tagTimestamp = preferredTag.timestamp;

    // if we have historical poses, use them to correct the vector
    // based on the difference between current and historic poses
    if (tagTimestamp > epoch && state.hasSampledPoses()) {
      final var poseThen = state.findPoseAt(tagTimestamp);
      final var poseNow = state.findPoseAt(timestamp);
      final var translationThen = poseThen.getTranslation();
      final var translationNow = poseNow.getTranslation();
      final var rotationThen = poseThen.getRotation();
      final var rotationNow = poseNow.getRotation();

      // convert from vision frame of reference to field frame of reference
      final var fieldTranslation = new Translation2d(-z, x).rotateBy(rotationThen);
      final var dx = translationNow.getX() - translationThen.getX();
      final var dy = translationNow.getY() - translationThen.getY();

      // vector to the vision target in field frame of reference
      final var targetVector =
          new Translation2d(fieldTranslation.getX() + dx, fieldTranslation.getY() + dy);
      Logger.recordOutput("TrackedShooting/Target", translationNow.plus(targetVector));

      // convert target vector back to the vision frame of reference
      final var t = targetVector.rotateBy(rotationNow.unaryMinus());
      x = t.getY();
      z = -t.getX();
    }

    final var distToTag1 = Math.floor(getHypotenus(z, Math.abs(x)) * 100) / 100;
    Logger.recordOutput("TrackedShooting/Tag1Dist", distToTag1);

    // -2.2m -- target height over ground
    final var pivotAngle = Math.abs(Math.toDegrees(Math.atan2(-2.0, distToTag1)));
    Logger.recordOutput("TrackedShooting/PivotAngle", pivotAngle);

    final var turnAngle = Math.atan2(x, z);
    Logger.recordOutput("TrackedShooting/TurnAngle", Math.toDegrees(turnAngle));

    // Calculate the interpolation between the two distance->speed values.
    // y0(x1-x)+y1(x-x0)
    // _________________
    //      x1-x0
    final var maxDist = 7.0;
    final var minDist = 2.0;
    final var maxSpeed = 4500;
    final var minSpeed = 2702;
    final var lerpSpeed =
        (minSpeed * (maxDist - distToTag1) + maxSpeed * (distToTag1 - minDist))
            / (maxDist - minDist);
    Logger.recordOutput("TrackedShooting/LerpSpeed", lerpSpeed);

    // consider tag acquired if we see one or we have seen it within the last 2 seconds
    // so that we can use recent odometry compensation to steer us towards it
    final var targetAcquired =
        preferredTag.tracked
            || (timestamp - preferredTag.timestamp) < 2 && !disableOdometryHistory.get();

    if (targetAcquired) {
      drive.steeringLock(turnAngle);
      pivot.setRequestedAngle(pivotAngle);
      shooter.setSpeed(lerpSpeed);

      if (pivot.isAtGoal()
          && shooter.isAtSpeed(lerpSpeed)
          && Math.abs((Math.toDegrees(turnAngle))) < 1) {
        goodToShoot = true;
        shooter.feedOut();
      } else {
        goodToShoot = false;
      }
    } else {
      drive.steeringUnlock();
    }

    Logger.recordOutput("TrackedShooting/GoodToShoot", goodToShoot);

    // runs active steering if needed in autonomous mode
    if (DriverStation.isAutonomous()) {
      drive.runVelocity(new ChassisSpeeds());
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.steeringUnlock();
    shooter.stop();
    pivot.setRequestedAngle(20);
  }

  @Override
  public boolean isFinished() {
    return !shooter.hasGamePiece();
  }

  private double getHypotenus(double a, double b) {
    return Math.sqrt(a * a + b * b);
  }
}
