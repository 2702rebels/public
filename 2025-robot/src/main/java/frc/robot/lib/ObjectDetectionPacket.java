package frc.robot.lib;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

/** A datagram structure communicated by the object detection code. */
public class ObjectDetectionPacket implements StructSerializable {
  /** Magic number */
  public final int key;

  /** Version */
  public final int version;

  /** Timestamp in microseconds (NetworkTable reference) */
  public final long timestamp;

  /** Numbers of object slots actually reported */
  public final byte count;

  /** Detected object slots (4) */
  public final ObjectDetectionInfo[] objects;

  public ObjectDetectionPacket(
      int key, int version, long timestamp, byte count, ObjectDetectionInfo[] objects) {
    if (objects == null || objects.length != 4) {
      throw new Error("Number of object slots must equal to 4");
    }

    this.key = key;
    this.version = version;
    this.timestamp = timestamp;
    this.count = count;
    this.objects = objects;
  }

  public static final ObjectDetectionPacketStruct struct = new ObjectDetectionPacketStruct();

  public static final class ObjectDetectionPacketStruct implements Struct<ObjectDetectionPacket> {
    @Override
    public Class<ObjectDetectionPacket> getTypeClass() {
      return ObjectDetectionPacket.class;
    }

    @Override
    public String getTypeName() {
      return "ObjectDetectionPacket";
    }

    @Override
    public int getSize() {
      return kSizeInt32 * 2 + kSizeInt64 + kSizeInt8 + ObjectDetectionInfo.struct.getSize() * 4;
    }

    @Override
    public String getSchema() {
      return "int key;int version;long timestamp;uint8 count;ObjectDetectionInfo objects[4];";
    }

    @Override
    public ObjectDetectionPacket unpack(ByteBuffer bb) {
      return new ObjectDetectionPacket(
          bb.getInt(),
          bb.getInt(),
          bb.getLong(),
          bb.get(),
          new ObjectDetectionInfo[] {
            ObjectDetectionInfo.struct.unpack(bb),
            ObjectDetectionInfo.struct.unpack(bb),
            ObjectDetectionInfo.struct.unpack(bb),
            ObjectDetectionInfo.struct.unpack(bb)
          });
    }

    @Override
    public void pack(ByteBuffer bb, ObjectDetectionPacket value) {
      bb.putInt(value.key);
      bb.putInt(value.version);
      bb.putLong(value.timestamp);
      bb.put(value.count);
      ObjectDetectionInfo.struct.pack(bb, value.objects[0]);
      ObjectDetectionInfo.struct.pack(bb, value.objects[1]);
      ObjectDetectionInfo.struct.pack(bb, value.objects[2]);
      ObjectDetectionInfo.struct.pack(bb, value.objects[3]);
    }
  }
}
