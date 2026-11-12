package frc.robot.lib;

public class VisionPose {
  /** Timestamp of last observation / update in microseconds. */
  public long timestamp;

  /** Last known X in meters. */
  public double x;

  /** Last known Y in meters. */
  public double y;

  /** Last known Theta in radians. */
  public double theta;

  /** Last known quality. */
  public double quality;
}
