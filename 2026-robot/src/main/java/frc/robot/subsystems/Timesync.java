package frc.robot.subsystems;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.lib.UdpListenerThread;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.ZoneOffset;
import java.util.Optional;

public class Timesync {
  static class PingPacket implements StructSerializable {
    /** Protocol version: should be set to 1 */
    public final byte protocolVersion;

    /** Message ID: should be set to 1 */
    public final byte messageId;

    /** Client's local time. This value is round-tripped. */
    public final long clientLocalTime;

    public PingPacket(byte protocolVersion, byte messageId, long clientLocalTime) {
      this.protocolVersion = protocolVersion;
      this.messageId = messageId;
      this.clientLocalTime = clientLocalTime;
    }

    public static final PingPacketStruct struct = new PingPacketStruct();

    public static final class PingPacketStruct implements Struct<PingPacket> {
      @Override
      public Class<PingPacket> getTypeClass() {
        return PingPacket.class;
      }

      @Override
      public String getTypeName() {
        return "PingPacket";
      }

      @Override
      public int getSize() {
        return kSizeInt8 * 2 + kSizeInt64;
      }

      @Override
      public String getSchema() {
        return "uint8 protocolVersion;uint8 messageId;uint64 clientLocalTime;";
      }

      @Override
      public PingPacket unpack(ByteBuffer bb) {
        return new PingPacket(bb.get(), bb.get(), bb.getLong());
      }

      @Override
      public void pack(ByteBuffer bb, PingPacket value) {
        bb.put(value.protocolVersion);
        bb.put(value.messageId);
        bb.putLong(value.clientLocalTime);
      }
    }
  }

  static class PongPacket implements StructSerializable {
    /** Protocol version: should be set to 1 */
    public final byte protocolVersion;

    /** Message ID: should be set to 2 */
    public final byte messageId;

    /** Client's local time. This value is round-tripped. */
    public final long clientLocalTime;

    /** Server's local time in microseconds. */
    public final long serverLocalTime;

    /** Server's clock time in seconds since Unix epoch. */
    public final long serverClockTime;

    /**
     * Match type if supplied by FMS (1 - practice, 2 - qualification, 3 - elimination), otherwise
     * 0.
     */
    public final byte matchType;

    /** Match number if supplied by FMS, otherwise 0. */
    public final byte matchNumber;

    public PongPacket(
        byte protocolVersion,
        byte messageId,
        long clientLocalTime,
        long serverLocalTime,
        long serverClockTime,
        byte matchType,
        byte matchNumber) {
      this.protocolVersion = protocolVersion;
      this.messageId = messageId;
      this.clientLocalTime = clientLocalTime;
      this.serverLocalTime = serverLocalTime;
      this.serverClockTime = serverClockTime;
      this.matchType = matchType;
      this.matchNumber = matchNumber;
    }

    public static final PongPacketStruct struct = new PongPacketStruct();

    public static final class PongPacketStruct implements Struct<PongPacket> {
      @Override
      public Class<PongPacket> getTypeClass() {
        return PongPacket.class;
      }

      @Override
      public String getTypeName() {
        return "PongPacket";
      }

      @Override
      public int getSize() {
        return kSizeInt8 * 4 + kSizeInt64 * 3;
      }

      @Override
      public String getSchema() {
        return "uint8 protocolVersion;uint8 messageId;uint64 clientLocalTime;uint64 serverLocalTime;uint64 serverClockTime;uint8 matchType;uint8 matchNumber;";
      }

      @Override
      public PongPacket unpack(ByteBuffer bb) {
        return new PongPacket(
            bb.get(), bb.get(), bb.getLong(), bb.getLong(), bb.getLong(), bb.get(), bb.get());
      }

      @Override
      public void pack(ByteBuffer bb, PongPacket value) {
        bb.put(value.protocolVersion);
        bb.put(value.messageId);
        bb.putLong(value.clientLocalTime);
        bb.putLong(value.serverLocalTime);
        bb.putLong(value.serverClockTime);
        bb.put(value.matchType);
        bb.put(value.matchNumber);
      }
    }
  }

  private final UdpListenerThread<PingPacket> listener;

  private static final ZoneId UTC = ZoneId.of("UTC");
  private static final byte kProtocolVersion = 1;
  private static final byte kPingMessageId = 1;
  private static final byte kPongMessageId = 2;

  public Timesync() {
    listener =
        new UdpListenerThread<PingPacket>(
            Constants.Network.UDP_PORT_TIMESYNC,
            1500,
            this::packetReceived,
            PingPacket.struct::unpack);

    listener.start();
  }

  private static byte getMatchTypeCode(MatchType type) {
    return switch (type) {
      case Practice -> 1;
      case Qualification -> 2;
      case Elimination -> 3;
      default -> 0;
    };
  }

  private Optional<byte[]> packetReceived(PingPacket packet) {
    // validate packet format, ignore malformed packets
    if (packet.protocolVersion != kProtocolVersion || packet.messageId != kPingMessageId) {
      return Optional.empty();
    }

    final var timestamp = RobotController.getTime();
    final var matchNumber = (byte) DriverStation.getMatchNumber();
    final var matchType = DriverStation.getMatchType();

    // clock time if supplied by the FMS/DS
    var utcSecondsSinceEpoch = 0L;
    if (RobotController.isSystemTimeValid()) {
      utcSecondsSinceEpoch = LocalDateTime.now(UTC).toEpochSecond(ZoneOffset.UTC);
    }

    // construct pong packet
    final var pong =
        new PongPacket(
            kProtocolVersion,
            kPongMessageId,
            packet.clientLocalTime,
            timestamp,
            utcSecondsSinceEpoch,
            getMatchTypeCode(matchType),
            matchNumber);

    // serialize to binary and respond
    final var bb = ByteBuffer.allocate(PongPacket.struct.getSize()).order(ByteOrder.LITTLE_ENDIAN);
    PongPacket.struct.pack(bb, pong);

    return Optional.of(bb.array());
  }
}
