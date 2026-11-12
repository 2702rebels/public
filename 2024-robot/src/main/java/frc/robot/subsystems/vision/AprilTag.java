package frc.robot.subsystems.vision;

public class AprilTag {
  public AprilTag(int id) {
    this.id = id;
  }

  /** Identifier of the tag. */
  public final int id;

  /** Timestamp of last observation / update. */
  public double timestamp;

  /** Indicates that tag is tracked and (X,Y,Z) can be trusted. */
  public boolean tracked;

  /** Last known X in meters. */
  public double x;

  /** Last known Y in meters. */
  public double y;

  /** Last known z in meters. */
  public double z;
}
