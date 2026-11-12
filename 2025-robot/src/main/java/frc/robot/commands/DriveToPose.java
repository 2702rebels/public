package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class DriveToPose extends Command {
  private final double minDistance = Units.inchesToMeters(6);
  private final double maxDistance = Units.inchesToMeters(24);

  private final Supplier<Optional<Pose2d>> targetPoseSupplier;
  private final Supplier<Pose2d> currentPoseSupplier;
  private final BooleanSupplier stopCondition;
  private final CommandSwerveDrivetrain drive;
  private boolean running;
  private Translation2d setpointTranslation;

  private final ProfiledPIDController distanceController;

  private final ProfiledPIDController rotationController =
      new ProfiledPIDController(
          DriveConstants.PoseChasing.rotationKp,
          DriveConstants.PoseChasing.rotationKi,
          DriveConstants.PoseChasing.rotationKd,
          new TrapezoidProfile.Constraints(
              DriveConstants.maxAngularRate.in(RadiansPerSecond) * 0.5,
              DriveConstants.maxAngularAcceleration.in(RadiansPerSecondPerSecond) * 0.5),
          Constants.cyclePeriodSeconds);

  private double distanceErrorAbs;
  private double rotationErrorAbs;

  public DriveToPose(
      CommandSwerveDrivetrain drive, Pose2d targetPose, Supplier<Pose2d> currentPoseSupplier) {
    this(
        drive,
        () -> Optional.of(targetPose),
        currentPoseSupplier,
        DriveConstants.maxSpeed.times(0.5),
        DriveConstants.maxAcceleration.times(0.5),
        null);
  }

  public DriveToPose(
      CommandSwerveDrivetrain drive,
      Supplier<Optional<Pose2d>> targetPoseSupplier,
      Supplier<Pose2d> currentPoseSupplier) {
    this(
        drive,
        targetPoseSupplier,
        currentPoseSupplier,
        DriveConstants.maxSpeed.times(0.5),
        DriveConstants.maxAcceleration.times(0.5),
        null);
  }

  public DriveToPose(
      CommandSwerveDrivetrain drive,
      Supplier<Optional<Pose2d>> targetPoseSupplier,
      Supplier<Pose2d> currentPoseSupplier,
      LinearVelocity maxVelocity,
      LinearAcceleration maxAcceleration) {
    this(drive, targetPoseSupplier, currentPoseSupplier, maxVelocity, maxAcceleration, null);
  }

  public DriveToPose(
      CommandSwerveDrivetrain drive,
      Supplier<Optional<Pose2d>> targetPoseSupplier,
      Supplier<Pose2d> currentPoseSupplier,
      LinearVelocity maxVelocity,
      LinearAcceleration maxAcceleration,
      BooleanSupplier stopCondition) {
    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
    this.currentPoseSupplier = currentPoseSupplier;
    this.stopCondition = stopCondition;
    addRequirements(drive);

    distanceController =
        new ProfiledPIDController(
            DriveConstants.PoseChasing.translationKp,
            DriveConstants.PoseChasing.translationKi,
            DriveConstants.PoseChasing.translationKd,
            new TrapezoidProfile.Constraints(
                maxVelocity.in(MetersPerSecond), maxAcceleration.in(MetersPerSecondPerSecond)),
            Constants.cyclePeriodSeconds);

    distanceController.setTolerance(Constants.DriveConstants.PoseChasing.kTranslationTolerance);
    rotationController.setTolerance(Constants.DriveConstants.PoseChasing.kRotationTolerance);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    final var pose = targetPoseSupplier.get();
    if (!pose.isPresent()) {
      return;
    }

    final var currentFieldVelocity = drive.getFieldSpeeds();
    final var currentPose = currentPoseSupplier.get();
    final var desiredPose = pose.get();

    final var currentTranslation = currentPose.getTranslation();
    final var desiredTranslation = desiredPose.getTranslation();

    final var distance = currentTranslation.getDistance(desiredTranslation);
    final var angle = currentTranslation.minus(desiredTranslation).getAngle();

    // reset all controllers
    distanceController.reset(
        distance,
        Math.min(
            0.0,
            -new Translation2d(
                    currentFieldVelocity.vxMetersPerSecond, currentFieldVelocity.vyMetersPerSecond)
                .rotateBy(angle.unaryMinus())
                .getX()));

    rotationController.reset(
        currentPose.getRotation().getRadians(), currentFieldVelocity.omegaRadiansPerSecond);

    setpointTranslation = currentTranslation;
  }

  @Override
  public void execute() {
    final var pose = targetPoseSupplier.get();
    if (!pose.isPresent()) {
      return;
    }

    running = true;

    final var currentPose = currentPoseSupplier.get();
    final var desiredPose = pose.get();

    final var currentTranslation = currentPose.getTranslation();
    final var desiredTranslation = desiredPose.getTranslation();
    final var currentRotation = currentPose.getRotation().getRadians();
    final var desiredRotation = desiredPose.getRotation().getRadians();

    final var distance = currentTranslation.getDistance(desiredTranslation);
    final var angle = currentTranslation.minus(desiredTranslation).getAngle();

    distanceErrorAbs = distance;
    rotationErrorAbs = Math.abs(currentRotation - desiredRotation);

    final double scaleFF =
        MathUtil.clamp((distance - minDistance) / (maxDistance - minDistance), 0.0, 1.0);

    distanceController.reset(
        setpointTranslation.getDistance(desiredTranslation),
        distanceController.getSetpoint().velocity);

    var driveVelocityScalar =
        distanceController.getSetpoint().velocity * scaleFF
            + distanceController.calculate(distance, 0.0);

    // stop driving when at goal
    if (distance <= distanceController.getPositionTolerance()) {
      driveVelocityScalar = 0.0;
    }

    setpointTranslation =
        desiredTranslation.plus(
            new Translation2d(distanceController.getSetpoint().position, 0.0).rotateBy(angle));

    var rotationVelocity =
        rotationController.getSetpoint().velocity
            + rotationController.calculate(currentRotation, desiredRotation);

    // stop rotating when at goal
    if (rotationErrorAbs <= rotationController.getPositionTolerance()) {
      rotationVelocity = 0.0;
    }

    final var driveVelocity = new Translation2d(driveVelocityScalar, 0.0).rotateBy(angle);

    drive.driveAtRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(),
            driveVelocity.getY(),
            rotationVelocity,
            currentPose.getRotation()));

    if (Constants.Settings.rioLoggingEnabled) {
      // spotless:off
      DogLog.log("DriveToPose/DistanceMeasured", distance);
      DogLog.log("DriveToPose/DistanceSetpoint", distanceController.getSetpoint().position);
      DogLog.log("DriveToPose/DistanceVelocity", distanceController.getSetpoint().velocity);
      DogLog.log("DriveToPose/RotationMeasured", Units.radiansToDegrees(currentRotation));
      DogLog.log("DriveToPose/RotationSetpoint", Units.radiansToDegrees(rotationController.getSetpoint().position));
      DogLog.log("DriveToPose/RotationVelocity", rotationController.getSetpoint().velocity);
      DogLog.log("DriveToPose/RotationError", Units.radiansToDegrees(rotationErrorAbs));
      DogLog.log("DriveToPose/DistanceAtGoal", distanceController.atGoal());
      DogLog.log("DriveToPose/RotationAtGoal", rotationController.atGoal());
      // spotless:on
    }
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return !targetPoseSupplier.get().isPresent()
        || atGoal()
        || (stopCondition != null && stopCondition.getAsBoolean());
  }

  public boolean atGoal() {
    return running && distanceController.atGoal() && rotationController.atGoal();
  }

  /** Checks if the robot pose is within the allowed translation and rotation tolerances. */
  public boolean withinTolerance(double distanceTolerance, Rotation2d rotationTolerance) {
    return running
        && distanceErrorAbs <= distanceTolerance
        && rotationErrorAbs <= rotationTolerance.getRadians();
  }

  public boolean isRunning() {
    return running;
  }
}
