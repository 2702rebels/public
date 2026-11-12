package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
  private static final LoggedTunableNumber driveP =
      new LoggedTunableNumber("Drive/Pose/Drive P", 1.5);
  private static final LoggedTunableNumber driveI =
      new LoggedTunableNumber("Drive/Pose/Drive I", 0.0);
  private static final LoggedTunableNumber driveD =
      new LoggedTunableNumber("Drive/Pose/Drive D", 0.0);
  private static final LoggedTunableNumber driveTol =
      new LoggedTunableNumber("Drive/Pose/Drive Tolerance", 0.05);
  private static final LoggedTunableNumber driveMaxV =
      new LoggedTunableNumber(
          "Drive/Pose/Drive Max V", DriveConstants.drivetrainConfig.maxLinearSpeed() * 0.8);
  private static final LoggedTunableNumber driveMaxA =
      new LoggedTunableNumber(
          "Drive/Pose/Drive Max A", DriveConstants.drivetrainConfig.maxLinearAcceleration() * 0.8);
  private static final LoggedTunableNumber thetaP =
      new LoggedTunableNumber("Drive/Pose/Theta P", 1.5);
  private static final LoggedTunableNumber thetaI =
      new LoggedTunableNumber("Drive/Pose/Theta I", 0.0);
  private static final LoggedTunableNumber thetaD =
      new LoggedTunableNumber("Drive/Pose/Theta D", 0.0);
  private static final LoggedTunableNumber thetaTol =
      new LoggedTunableNumber("Drive/Pose/Theta Tolerance", Units.degreesToRadians(2));
  private static final LoggedTunableNumber thetaMaxV =
      new LoggedTunableNumber(
          "Drive/Pose/Theta Max V", DriveConstants.drivetrainConfig.maxAngularSpeed());
  private static final LoggedTunableNumber thetaMaxA =
      new LoggedTunableNumber(
          "Drive/Pose/Theta Max A", DriveConstants.drivetrainConfig.maxAngularAcceleration());
  private static final LoggedTunableNumber ffMinDistance =
      new LoggedTunableNumber("Drive/Pose/FF Min Distance", 0.2);
  private static final LoggedTunableNumber ffMaxDistance =
      new LoggedTunableNumber("Drive/Pose/FF Max Distance", 0.8);

  protected Supplier<Pose2d> poseSupplier;
  private final boolean stopAtGoal;
  private final BooleanSupplier stopCondition;
  private final Drive drive;
  private boolean running;
  private Translation2d lastSetpointTranslation;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          driveP.get(),
          driveI.get(),
          driveD.get(),
          new TrapezoidProfile.Constraints(driveMaxV.get(), driveMaxA.get()),
          Constants.cyclePeriodSeconds);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetaP.get(),
          thetaI.get(),
          thetaD.get(),
          new TrapezoidProfile.Constraints(thetaMaxV.get(), thetaMaxA.get()),
          Constants.cyclePeriodSeconds);
  private double driveErrorAbs;
  private double thetaErrorAbs;

  public DriveToPose(Drive drive, Pose2d pose) {
    this(drive, () -> pose, true, null);
  }

  public DriveToPose(
      Drive drive,
      Supplier<Pose2d> poseSupplier,
      boolean stopAtGoal,
      BooleanSupplier stopCondition) {
    this.drive = drive;
    this.poseSupplier = poseSupplier;
    this.stopAtGoal = stopAtGoal;
    this.stopCondition = stopCondition;
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    driveController.setTolerance(driveTol.get());
    thetaController.setTolerance(thetaTol.get());

    // reset all controllers
    final var currentPose = drive.getPose();
    final var desiredPose = poseSupplier.get();

    final var currentDistance =
        currentPose.getTranslation().getDistance(desiredPose.getTranslation());

    final var currentAngle =
        currentPose.getTranslation().minus(desiredPose.getTranslation()).getAngle();

    final var currentFieldVelocity = drive.getFieldVelocity();

    driveController.reset(
        currentDistance,
        Math.min(
            0.0,
            -new Translation2d(currentFieldVelocity.dx, currentFieldVelocity.dy)
                .rotateBy(currentAngle.unaryMinus())
                .getX()));

    thetaController.reset(currentPose.getRotation().getRadians(), drive.getYawVelocityRadPerSec());
    lastSetpointTranslation = drive.getPose().getTranslation();
  }

  @Override
  public void execute() {
    running = true;
    final var callerId = hashCode();

    // update controllers if tunable numbers have changed
    if (driveMaxV.hasChanged(callerId)
        || driveMaxA.hasChanged(callerId)
        || driveTol.hasChanged(callerId)
        || driveP.hasChanged(callerId)
        || driveI.hasChanged(callerId)
        || driveD.hasChanged(callerId)
        || thetaMaxV.hasChanged(callerId)
        || thetaMaxA.hasChanged(callerId)
        || thetaTol.hasChanged(callerId)
        || thetaP.hasChanged(callerId)
        || thetaI.hasChanged(callerId)
        || thetaD.hasChanged(callerId)) {
      driveController.setP(driveP.get());
      driveController.setI(driveI.get());
      driveController.setD(driveD.get());
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(driveMaxV.get(), driveMaxA.get()));
      driveController.setTolerance(driveTol.get());
      thetaController.setP(thetaP.get());
      thetaController.setI(thetaI.get());
      thetaController.setD(thetaD.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(thetaMaxV.get(), thetaMaxA.get()));
      thetaController.setTolerance(thetaTol.get());
    }

    final var currentPose = drive.getPose();
    final var desiredPose = poseSupplier.get();
    final var currentTranslation = currentPose.getTranslation();
    final var desiredTranslation = desiredPose.getTranslation();
    final var currentTheta = currentPose.getRotation().getRadians();
    final var desiredTheta = desiredPose.getRotation().getRadians();

    final var currentDistance = currentTranslation.getDistance(desiredTranslation);
    final var currentAngle = currentTranslation.minus(desiredTranslation).getAngle();

    driveErrorAbs = currentDistance;
    final var minDistance = ffMinDistance.get();
    final var maxDistance = ffMaxDistance.get();

    final double scaleFF =
        MathUtil.clamp((currentDistance - minDistance) / (maxDistance - minDistance), 0.0, 1.0);

    driveController.reset(
        lastSetpointTranslation.getDistance(desiredPose.getTranslation()),
        driveController.getSetpoint().velocity);

    var driveVelocityScalar =
        driveController.getSetpoint().velocity * scaleFF
            + driveController.calculate(driveErrorAbs, 0.0);

    // stop driving when at goal
    if (currentDistance < driveController.getPositionTolerance()) {
      driveVelocityScalar = 0.0;
    }

    lastSetpointTranslation =
        desiredPose
            .getTranslation()
            .plus(
                new Translation2d(driveController.getSetpoint().position, 0.0)
                    .rotateBy(currentAngle));

    var thetaVelocity =
        thetaController.getSetpoint().velocity * MathUtil.clamp(scaleFF, 0.4, 0.8)
            + thetaController.calculate(currentTheta, desiredTheta);

    thetaErrorAbs = Math.abs(currentTheta - desiredTheta);

    // stop rotating when at goal
    if (thetaErrorAbs < thetaController.getPositionTolerance()) {
      thetaVelocity = 0.0;
    }

    final var driveVelocity = new Translation2d(driveVelocityScalar, 0.0).rotateBy(currentAngle);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

    Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.recordOutput("DriveToPose/ThetaMeasured", currentTheta);
    Logger.recordOutput("DriveToPose/ThetaDesired", desiredTheta);
    Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput("DriveToPose/ThetaError", thetaErrorAbs);
    Logger.recordOutput("DriveToPose/ThetaVelocity", thetaController.getSetpoint().velocity);
    Logger.recordOutput(
        "DriveToPose/Setpoint",
        new Pose2d(
            lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
    Logger.recordOutput("DriveToPose/Goal", desiredPose);
    Logger.recordOutput("DriveToPose/DriveAtGoal", driveController.atGoal());
    Logger.recordOutput("DriveToPose/ThetaAtGoal", thetaController.atGoal());
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return stopAtGoal && atGoal() && (stopCondition == null || stopCondition.getAsBoolean());
  }

  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }

  public boolean isRunning() {
    return running;
  }
}
