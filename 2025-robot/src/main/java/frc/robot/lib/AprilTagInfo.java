package frc.robot.lib;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

/** Describes detected AprilTag. */
public class AprilTagInfo implements StructSerializable {
  /** Identifier of the tag */
  public final byte id;

  /** X component of AprilTag vector in the robot frame of reference (inches) */
  public final double x;

  /** Y component of AprilTag vector in the robot frame of reference (inches) */
  public final double y;

  public AprilTagInfo(byte id, double x, double y) {
    this.id = id;
    this.x = x;
    this.y = y;
  }

  public static final AprilTagInfoStruct struct = new AprilTagInfoStruct();

  public static final class AprilTagInfoStruct implements Struct<AprilTagInfo> {
    @Override
    public Class<AprilTagInfo> getTypeClass() {
      return AprilTagInfo.class;
    }

    @Override
    public String getTypeName() {
      return "AprilTagInfo";
    }

    @Override
    public int getSize() {
      return kSizeInt8 + kSizeDouble * 2;
    }

    @Override
    public String getSchema() {
      return "uint8 id;double x;double y;";
    }

    @Override
    public AprilTagInfo unpack(ByteBuffer bb) {
      return new AprilTagInfo(bb.get(), bb.getDouble(), bb.getDouble());
    }

    @Override
    public void pack(ByteBuffer bb, AprilTagInfo value) {
      bb.put(value.id);
      bb.putDouble(value.x);
      bb.putDouble(value.y);
    }
  }
}
