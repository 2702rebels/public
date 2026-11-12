package frc.robot.lib;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

/** Describes detected object. */
public class ObjectDetectionInfo implements StructSerializable {
  /** Type of the detected object */
  public final byte type;

  /** Arbitrary quality value */
  public final byte quality;

  /** Distance measured in meters in the robot frame of reference */
  public final double distance;

  /** Angle in radians in the robot frame of reference as per FRC conventions */
  public final double angle;

  public ObjectDetectionInfo(byte type, double distance, double angle, byte quality) {
    this.type = type;
    this.distance = distance;
    this.angle = angle;
    this.quality = quality;
  }

  public static final ObjectDetectionInfoStruct struct = new ObjectDetectionInfoStruct();

  public static final class ObjectDetectionInfoStruct implements Struct<ObjectDetectionInfo> {
    @Override
    public Class<ObjectDetectionInfo> getTypeClass() {
      return ObjectDetectionInfo.class;
    }

    @Override
    public String getTypeName() {
      return "ObjectDetectionInfo";
    }

    @Override
    public int getSize() {
      return kSizeDouble * 2 + kSizeInt8 * 2;
    }

    @Override
    public String getSchema() {
      return "uint8 type;double distance;double angle;uint8 quality;";
    }

    @Override
    public ObjectDetectionInfo unpack(ByteBuffer bb) {
      return new ObjectDetectionInfo(bb.get(), bb.getDouble(), bb.getDouble(), bb.get());
    }

    @Override
    public void pack(ByteBuffer bb, ObjectDetectionInfo value) {
      bb.put(value.type);
      bb.putDouble(value.distance);
      bb.putDouble(value.angle);
      bb.put(value.quality);
    }
  }
}
