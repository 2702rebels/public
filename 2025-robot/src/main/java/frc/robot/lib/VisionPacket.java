package frc.robot.lib;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

/** A datagram structure communicated by the vision (AprilTag) detection code. */
public class VisionPacket implements StructSerializable {
  /** Flags */
  public final byte flags;

  /** Timestamp in microseconds (NetworkTable reference) */
  public final long timestamp;

  /** Estimated robot X position in the field frame of reference (inches) */
  public final double x;

  /** Estimated robot Y position in the field frame of reference (inches) */
  public final double y;

  /** Estimated robot rotation in the field frame of reference (radians) */
  public final double theta;

  /** Arbitrary quality value */
  public final double quality;

  /** Numbers of AprilTag slots actually reported */
  public final byte count;

  /** Detected AprilTag slots (6) */
  public final AprilTagInfo[] tags;

  public VisionPacket(
      byte flags,
      long timestamp,
      double x,
      double y,
      double theta,
      double quality,
      byte count,
      AprilTagInfo[] tags) {
    if (tags == null || tags.length != 6) {
      throw new Error("Number of tag slots must equal to 6");
    }

    this.flags = flags;
    this.timestamp = timestamp;
    this.x = x;
    this.y = y;
    this.theta = theta;
    this.quality = quality;
    this.count = count;
    this.tags = tags;
  }

  public static final VisionPacketStruct struct = new VisionPacketStruct();

  public static final class VisionPacketStruct implements Struct<VisionPacket> {
    @Override
    public Class<VisionPacket> getTypeClass() {
      return VisionPacket.class;
    }

    @Override
    public String getTypeName() {
      return "VisionPacket";
    }

    @Override
    public int getSize() {
      return kSizeInt8 * 2 + kSizeInt64 + kSizeDouble * 4 + AprilTagInfo.struct.getSize() * 6;
    }

    @Override
    public String getSchema() {
      return "uint8 flags;long timestamp;double x;double y;double theta;double quality;uint8 count;AprilTagInfo tags[6];";
    }

    @Override
    public VisionPacket unpack(ByteBuffer bb) {
      return new VisionPacket(
          bb.get(),
          bb.getLong(),
          bb.getDouble(),
          bb.getDouble(),
          bb.getDouble(),
          bb.getDouble(),
          bb.get(),
          new AprilTagInfo[] {
            AprilTagInfo.struct.unpack(bb),
            AprilTagInfo.struct.unpack(bb),
            AprilTagInfo.struct.unpack(bb),
            AprilTagInfo.struct.unpack(bb),
            AprilTagInfo.struct.unpack(bb),
            AprilTagInfo.struct.unpack(bb)
          });
    }

    @Override
    public void pack(ByteBuffer bb, VisionPacket value) {
      bb.put(value.flags);
      bb.putLong(value.timestamp);
      bb.putDouble(value.x);
      bb.putDouble(value.y);
      bb.putDouble(value.theta);
      bb.putDouble(value.quality);
      bb.put(value.count);
      AprilTagInfo.struct.pack(bb, value.tags[0]);
      AprilTagInfo.struct.pack(bb, value.tags[1]);
      AprilTagInfo.struct.pack(bb, value.tags[2]);
      AprilTagInfo.struct.pack(bb, value.tags[3]);
      AprilTagInfo.struct.pack(bb, value.tags[4]);
      AprilTagInfo.struct.pack(bb, value.tags[5]);
    }
  }
}
