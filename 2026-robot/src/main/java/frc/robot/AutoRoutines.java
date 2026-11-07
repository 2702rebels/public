package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

public class AutoRoutines {
  private final RobotState state;
  private final RobotContainer container;
  private final CommandSwerveDrivetrain drivetrain;
  private final FollowPath.Builder pathBuilder;

  private final NetworkTable auto = NetworkTableInstance.getDefault().getTable("Auto");
  private final Map<String, StructPublisher<Pose2d>> poseLoggers = new LinkedHashMap<>();

  private final PIDController translationController =
      new PIDController(
          DriveConstants.Translation.kP,
          DriveConstants.Translation.kI,
          DriveConstants.Translation.kD,
          Constants.cyclePeriodSeconds);

  private final PIDController rotationController =
      new PIDController(
          DriveConstants.Rotation.kP,
          DriveConstants.Rotation.kI,
          DriveConstants.Rotation.kD,
          Constants.cyclePeriodSeconds);

  private final PIDController crossTrackController =
      new PIDController(
          DriveConstants.CrossTrack.kP,
          DriveConstants.CrossTrack.kI,
          DriveConstants.CrossTrack.kD,
          Constants.cyclePeriodSeconds);

  private void logPose(String key, Pose2d pose) {
    var logger = poseLoggers.get(key);
    if (logger == null) {
      logger = auto.getStructTopic(key, Pose2d.struct).publish();
      poseLoggers.put(key, logger);
    }

    logger.accept(pose);
  }

  private Command shoot(Time duration) {
    return container
        .shooter
        .enable()
        .alongWith(container.hopper.enable(container.canShoot))
        .withTimeout(duration)
        .asProxy();
  }

  private Command climbExtend() {
    return container.climb.extend().asProxy();
  }

  private Command climbEngage() {
    return container.climb.engage().asProxy();
  }

  private Command intakeStart() {
    return container.intake.enable().asProxy();
  }

  private Command intakeStop() {
    return container.intake.disable().asProxy();
  }

  private Command intakeExtend() {
    return container.intake.extend().asProxy();
  }

  private Command intakeRetract() {
    return container.intake.retract().asProxy();
  }

  private Command intakeAgitate(Time duration) {
    return container.intake.agitate().withTimeout(duration).asProxy();
  }

  private Command intakeAgitateAndRetract(Time duration) {
    return Commands.sequence(intakeAgitate(duration), intakeRetract());
  }

  private Command intakeAgitate2() {
    return Commands.sequence(
        intakeAgitate(Seconds.of(1)),
        Commands.waitSeconds(0.5),
        intakeAgitate(Seconds.of(1)),
        Commands.waitSeconds(0.5),
        intakeAgitate(Seconds.of(1)));
  }

  private Command burp(Time duration) {
    return container.intake.burp().withTimeout(duration).asProxy();
  }

  private final Path trenchLBump2Runs = new Path("Trench L Bump 2 Runs");
  private final Path trenchRBump2Runs; // mirrored from [l]eft version

  private final Path trenchLBump2RunsFast = new Path("Trench L Bump 2 Runs Fast");
  private final Path trenchRBump2RunsFast; // mirrored from [l]eft version

  private final Path trenchLBump2RunsReckless = new Path("Trench L Bump 2 Runs Reckless");
  private final Path trenchRBump2RunsReckless; // mirrored from [l]eft version

  private final Path trenchLBump2RunsClimb = new Path("Trench L Bump 2 Runs Climb");
  private final Path trenchRBump2RunsClimb = new Path("Trench R Bump 2 Runs Climb");

  public AutoRoutines(RobotContainer container, CommandSwerveDrivetrain drivetrain) {
    this.container = container;
    this.drivetrain = drivetrain;
    this.state = RobotState.getInstance();

    // enable logging [development only]
    // if (Constants.DeploymentMode == Deployment.kDevelopment) {
    //   FollowPath.setPoseLoggingConsumer(pair -> logPose(pair.getFirst(), pair.getSecond()));
    // }

    // events
    FollowPath.registerEventTrigger("climb_extend", climbExtend());
    FollowPath.registerEventTrigger("climb_engage", climbEngage());
    FollowPath.registerEventTrigger("intake_start", intakeStart());
    FollowPath.registerEventTrigger("intake_stop", intakeStop());
    FollowPath.registerEventTrigger("intake_extend", intakeExtend());
    FollowPath.registerEventTrigger("intake_retract", intakeRetract());
    FollowPath.registerEventTrigger("intake_agitate", intakeAgitate(Seconds.of(4)));
    FollowPath.registerEventTrigger(
        "intake_agitate_retract", intakeAgitateAndRetract(Seconds.of(4)));
    FollowPath.registerEventTrigger("intake_agitate_2", intakeAgitate2());
    FollowPath.registerEventTrigger("burp", burp(Seconds.of(1)));
    FollowPath.registerEventTrigger("shoot_4", shoot(Seconds.of(4)));
    FollowPath.registerEventTrigger("shoot_5", shoot(Seconds.of(5)));
    FollowPath.registerEventTrigger("shoot_6", shoot(Seconds.of(6.5)));

    Path.setDefaultGlobalConstraints(
        new Path.DefaultGlobalConstraints(
            4, // maximum velocity (m/s)
            9.0, // maximum acceleration (m/s^2)
            540, // max angular velocity (deg/s)
            1080, // max angular acceleration (deg/s^2)
            DriveConstants.kTranslationTolerance, // translation tolerance (meters)
            DriveConstants.kRotationTolerance, // rotation tolerance (degrees)
            0.2 // handoff radius (meters)
            ));

    this.pathBuilder =
        new FollowPath.Builder(
                drivetrain,
                drivetrain::getPose,
                drivetrain::getRobotSpeeds,
                drivetrain::driveAtRobotRelative,
                translationController,
                rotationController,
                crossTrackController)
            .withDefaultShouldFlip();

    trenchRBump2Runs = trenchLBump2Runs.copy();
    trenchRBump2Runs.mirror();

    trenchRBump2RunsFast = trenchLBump2RunsFast.copy();
    trenchRBump2RunsFast.mirror();

    trenchRBump2RunsReckless = trenchLBump2RunsReckless.copy();
    trenchRBump2RunsReckless.mirror();
  }

  public Command trenchLBump2Runs() {
    return pathBuilder.build(trenchLBump2Runs);
  }

  public Command trenchRBump2Runs() {
    return pathBuilder.build(trenchRBump2Runs);
  }

  public Command trenchLBump2RunsFast() {
    return pathBuilder.build(trenchLBump2RunsFast);
  }

  public Command trenchRBump2RunsFast() {
    return pathBuilder.build(trenchRBump2RunsFast);
  }

  public Command trenchLBump2RunsReckless() {
    return pathBuilder.build(trenchLBump2RunsReckless);
  }

  public Command trenchRBump2RunsReckless() {
    return pathBuilder.build(trenchRBump2RunsReckless);
  }

  public Command trenchLBump2RunsClimb() {
    return pathBuilder
        .build(trenchLBump2RunsClimb)
        .withTimeout(Seconds.of(18.6))
        .andThen(climbEngage());
  }

  public Command trenchRBump2RunsClimb() {
    return pathBuilder
        .build(trenchRBump2RunsClimb)
        .withTimeout(Seconds.of(18.6))
        .andThen(climbEngage());
  }

  private static enum TowerPathSide {
    kRight,
    kLeft,
  }

  // tower left (upper) side
  private static final Path.Waypoint towerLeftEndpoint =
      new Path.Waypoint(new Translation2d(1.090, 4.60), Rotation2d.fromDegrees(90));

  private static final Path.Waypoint towerLeftApproach =
      new Path.Waypoint(
          towerLeftEndpoint.translationTarget().translation().minus(new Translation2d(0.2, 0)),
          Rotation2d.fromDegrees(90));

  // tower right (lower) side
  private static final Path.Waypoint towerRightEndpoint =
      new Path.Waypoint(new Translation2d(1.08, 2.88), Rotation2d.fromDegrees(-90));

  private static final Path.Waypoint towerRightApproach =
      new Path.Waypoint(
          towerRightEndpoint.translationTarget().translation().plus(new Translation2d(0.2, 0)),
          Rotation2d.fromDegrees(-90));

  private static final Path.RangedConstraint towerPathConstraint1 =
      new Path.RangedConstraint(0.4, 1, Integer.MAX_VALUE);

  private static final double kTimeToClimbApproachSeconds = 1.25;

  private static Path createTowerPath(TowerPathSide side, Pose2d pose) {
    // the pose is already transformed (flipped) if necessary
    final var poseXY = pose.getTranslation();

    // determine max speed based on how far are we from the approach endpoint
    final var approach = side == TowerPathSide.kRight ? towerRightApproach : towerLeftApproach;
    final var endpoint = side == TowerPathSide.kRight ? towerRightEndpoint : towerLeftEndpoint;

    final var distance = poseXY.getDistance(approach.translationTarget().translation());
    final var maxVelocity = MathUtil.clamp(distance / kTimeToClimbApproachSeconds, 0.5, 2.0);

    final var constraints =
        new Path.PathConstraints()
            .setMaxVelocityMetersPerSec(
                new Path.RangedConstraint(maxVelocity, 0, 1), towerPathConstraint1);

    return new Path(
        constraints,
        new Path.Waypoint(poseXY, approach.rotationTarget().rotation()),
        approach,
        endpoint);
  }

  /** Runs automatic climbing sequence if the robot is within the alliance zone. */
  public Command climb() {
    return Commands.defer(
        () -> {
          // must be relatively close to the tower to initiate the sequence
          final var pose = drivetrain.getPose();
          if (state.getFieldZone() != Field.FieldZone.kAlliance) {
            return Commands.none();
          }

          // select left/right (upper/lower) of the tower to climb
          final var towerY = Field.kTowerCenter.getY();
          final var robotY = pose.getY();

          final var side =
              Field.shouldFlip()
                  ? robotY > Field.flipY(towerY) ? TowerPathSide.kRight : TowerPathSide.kLeft
                  : robotY > towerY ? TowerPathSide.kLeft : TowerPathSide.kRight;

          // flip pose before giving it to the path, since it will auto-flip entire path
          return climbExtend()
              .andThen(
                  pathBuilder
                      .build(createTowerPath(side, Field.transform(pose)))
                      .withTimeout(Seconds.of(3)))
              .andThen(climbEngage());
        },
        Set.of(drivetrain));
  }
}
