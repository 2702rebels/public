package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionState {
  private static final TimestampedTranslation2d[] noTags = new TimestampedTranslation2d[0];

  /** Empty vision state. */
  public static final VisionState empty =
      new VisionState(new TimestampedValue<>(-1, Pose2d.kZero), noTags);

  /** Robot's pose in the field frame of reference */
  public final TimestampedValue<Pose2d> pose;

  /** AprilTag directions in the robot frame of reference */
  public final TimestampedTranslation2d[] tags;

  public VisionState(TimestampedValue<Pose2d> pose, TimestampedTranslation2d[] tags) {
    this.pose = pose;
    this.tags = tags;
  }

  public VisionState(TimestampedValue<Pose2d> pose) {
    this(pose, noTags);
  }
}
