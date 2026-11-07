package frc.robot.lib;

public class AprilTagPose {
  public AprilTagPose(byte id) {
    this.id = id;
  }

  /** Identifier of the tag. */
  public final byte id;

  /** Timestamp of last observation / update in microseconds. */
  public long timestamp;

  /** Last known X in meters in robot's frame of reference. */
  public double x;

  /** Last known Y in meters in robot's frame of reference. */
  public double y;
}
