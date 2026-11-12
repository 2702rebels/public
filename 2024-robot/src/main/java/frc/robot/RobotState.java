package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.TimestampedCircularBuffer;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  private final Kinematics<?, SwerveDriveWheelPositions> kinematics;

  // odometry is sampled at 5 ms (see Constants.odometryFrequency),
  // we typically see vision delays between 50-150 ms, therefore we
  // should track at least last 40 sampled poses for vision correction
  private TimestampedCircularBuffer<Pose2d> highFrequencySampledPoses =
      new TimestampedCircularBuffer<>(40);

  // we also keep track odometry poses at a lower sampling frequency of 20ms,
  // to allow for a bigger window (5 seconds) and enable longer target tracking
  private TimestampedCircularBuffer<Pose2d> lowFrequencySampledPoses =
      new TimestampedCircularBuffer<>((int) (5.0 / Constants.cyclePeriodSeconds));

  private Pose2d odometryPose = new Pose2d();
  private Pose2d estimatedPose = new Pose2d();

  // indicates that tags are being tracked
  private boolean isTrackingTags = false;

  private static final double poseBufferSizeSeconds = 2.0;
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);

  // fudge factor to multiple the maximum velocity and omega by,
  // when checking whether currently estimated deltas are outside
  // of reasonable range, resulting in observation being dropped
  private static final double deltaOverflowFactor = 3;
  private final double maxAcceptableVelocityMetersPerSecond;
  private final double maxAcceptableOmegaRadiansPerSecond;

  private double prevTimestamp = -1;
  private Rotation2d prevGyroAngle = new Rotation2d();
  private SwerveDriveWheelPositions prevWheelPositions =
      new SwerveDriveWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          });

  private RobotState() {
    kinematics = DriveConstants.kinematics;
    maxAcceptableVelocityMetersPerSecond =
        DriveConstants.drivetrainConfig.maxLinearSpeed() * deltaOverflowFactor;
    maxAcceptableOmegaRadiansPerSecond =
        DriveConstants.drivetrainConfig.turnMotorMaxAngularSpeed() * deltaOverflowFactor;
  }

  /** Timestamp when auto mode started. */
  public double autoStartedAt;

  /** Adds new odometry observation and updates robot pose. */
  public void addOdometryObservation(
      SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, double timestamp) {
    // if we have previous measurement available estimate the velocities based
    // on the difference in position and time; drop the observation if the estimated
    // values exceed physical limits
    if (prevTimestamp > 0 && prevTimestamp < timestamp) {
      final var dt = timestamp - prevTimestamp;

      for (var i = 0; i < wheelPositions.positions.length; ++i) {
        final var distance =
            wheelPositions.positions[i].distanceMeters
                - prevWheelPositions.positions[i].distanceMeters;

        final var angle =
            wheelPositions.positions[i].angle.getRadians()
                - prevWheelPositions.positions[i].angle.getRadians();

        final var velocity = distance / dt;
        final var omega = angle / dt;

        if (Math.abs(velocity) >= maxAcceptableVelocityMetersPerSecond
            || Math.abs(omega) >= maxAcceptableOmegaRadiansPerSecond) {
          return;
        }
      }
    }

    prevTimestamp = timestamp;
    final var prevOdometryPose = odometryPose;

    // the twist represents the motion of the robot since the last
    // sample in x, y, and theta based only on the modules, without
    // the gyro; the gyro is always disconnected in simulation
    var twist = kinematics.toTwist2d(prevWheelPositions, wheelPositions);

    // populate previous wheel positions (no extra allocations)
    for (var i = 0; i < wheelPositions.positions.length; ++i) {
      prevWheelPositions.positions[i].distanceMeters = wheelPositions.positions[i].distanceMeters;
      prevWheelPositions.positions[i].angle = wheelPositions.positions[i].angle;
    }

    // update dtheta for twist if gyro is connected and angle is provided
    if (gyroAngle != null) {
      twist.dtheta = gyroAngle.getRadians() - prevGyroAngle.getRadians();
      prevGyroAngle = gyroAngle;
    }

    // apply twist to the current odometry pose
    odometryPose = odometryPose.exp(twist);

    // add pose to buffer at timestamp
    poseBuffer.addSample(timestamp, odometryPose);

    // calculate diff from last odometry pose and add onto pose estimate
    estimatedPose = estimatedPose.exp(prevOdometryPose.log(odometryPose));

    // history of recently sampled poses
    highFrequencySampledPoses.enqueue(timestamp, estimatedPose);

    if (lowFrequencySampledPoses.getHeadTimestamp() <= timestamp - Constants.cyclePeriodSeconds) {
      lowFrequencySampledPoses.enqueue(timestamp, estimatedPose);
    }
  }

  /** Resets pose. */
  public void resetPose(Pose2d initialPose) {
    estimatedPose = initialPose;
    odometryPose = initialPose;
    poseBuffer.clear();
  }

  /** Returns one of the recently sampled poses with the timestamp closest to the specified one. */
  public Pose2d findPoseAt(double timestamp) {
    return timestamp < highFrequencySampledPoses.getTailTimestamp()
        ? lowFrequencySampledPoses.findAt(timestamp)
        : highFrequencySampledPoses.findAt(timestamp);
  }

  /** Determines whether the specified timestamp is within the tracked odometry range. */
  public boolean withinTrackedOdometry(double timestamp) {
    return timestamp >= highFrequencySampledPoses.getTailTimestamp();
  }

  /** Returns the current (estimated) robot pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/OdometryPose")
  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  /** Returns the current wheel positions. */
  public SwerveDriveWheelPositions getWheelPositions() {
    return prevWheelPositions;
  }

  /** Determines whether we have any sampled poses. */
  public boolean hasSampledPoses() {
    return highFrequencySampledPoses.size() > 0;
  }

  /** Updates tags tracking status. */
  public void setTrackingTags(boolean value) {
    isTrackingTags = value;
  }

  /** Determines whether tags are being tracked. */
  @AutoLogOutput(key = "Odometry/TagsTracking")
  public boolean isTrackingTags() {
    return this.isTrackingTags;
  }

  /** Determines whether robot is over the center line based on estimated pose. */
  public boolean isOverCenterLine() {
    final var red =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    final var x = getEstimatedPose().getTranslation().getX();
    final var r = DriveConstants.drivetrainConfig.driveBaseRadius();
    final var cx = FieldConstants.fieldCenterMeters;

    return red ? x <= (cx - r) : x >= (cx + r);
  }
}
