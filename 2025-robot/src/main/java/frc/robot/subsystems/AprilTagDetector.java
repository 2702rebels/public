package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.lib.AprilTagPose;
import frc.robot.lib.TimestampedTranslation2d;
import frc.robot.lib.TimestampedValue;
import frc.robot.lib.UdpListenerThread;
import frc.robot.lib.VisionPacket;
import frc.robot.lib.VisionPose;
import frc.robot.lib.VisionState;
import java.util.concurrent.atomic.AtomicLong;

public class AprilTagDetector {
  private final UdpListenerThread<VisionPacket> listener;
  private AtomicLong latency = new AtomicLong(0);

  private final AprilTagPose[] tags = new AprilTagPose[22]; // 22 april tags locations
  private VisionPose pose = new VisionPose();

  private static long kTimeSyncCutoffMicroseconds = 2000000; // 2 seconds

  private static final TimestampedValue<Pose2d> noPose = new TimestampedValue<>(-1, Pose2d.kZero);
  private static final TimestampedTranslation2d noTag =
      new TimestampedTranslation2d(-1, Translation2d.kZero);

  public AprilTagDetector() {
    listener =
        new UdpListenerThread<VisionPacket>(
            Constants.Network.UDP_PORT_APRILTAG_DETECTOR,
            1500,
            this::packetReceived,
            VisionPacket.struct::unpack);

    // initialize tags
    for (var i = 0; i < tags.length; ++i) {
      tags[i] = new AprilTagPose((byte) (i + 1));
    }

    listener.start();
  }

  private void packetReceived(VisionPacket packet) {
    final var t = RobotController.getFPGATime();

    // ignore packets that are too far into the past or into the future
    // indicating time sync issues
    if (Math.abs(t - packet.timestamp) > kTimeSyncCutoffMicroseconds) {
      return;
    }

    latency.set(t - packet.timestamp);

    synchronized (this) {
      // bit 1 indicates that pose can be trusted
      if ((packet.flags & 0x1) != 0 && packet.timestamp > pose.timestamp) {
        pose.timestamp = packet.timestamp;
        pose.x = Units.inchesToMeters(packet.x);
        pose.y = Units.inchesToMeters(packet.y);
        pose.theta = packet.theta;
        pose.quality = packet.quality;
      }

      if (packet.count > 0) {
        for (var i = 0; i < packet.count; ++i) {
          final var index = packet.tags[i].id - 1;
          tags[index].timestamp = packet.timestamp;
          tags[index].x = Units.inchesToMeters(packet.tags[i].x);
          tags[index].y = Units.inchesToMeters(packet.tags[i].y);
        }
      }
    }
  }

  /** Returns recent latency in milliseconds. */
  @Logged(name = "LatencyMS", importance = Importance.DEBUG)
  public double getLatency() {
    return latency.get() / 1000.0;
  }

  /** Returns snapshot of the vision state without AprilTags. */
  public synchronized VisionState getState() {
    return getState(false);
  }

  /** Returns snapshot of the vision state. */
  public synchronized VisionState getState(boolean includeAprilTags) {
    final var estimatedPose =
        pose.timestamp <= 0
            ? noPose
            : new TimestampedValue<>(
                pose.timestamp, new Pose2d(pose.x, pose.y, Rotation2d.fromRadians(pose.theta)));

    if (includeAprilTags) {
      final var estimatedTags = new TimestampedTranslation2d[22];
      for (var i = 0; i < estimatedTags.length; ++i) {
        estimatedTags[i] =
            tags[i].timestamp <= 0
                ? noTag
                : new TimestampedTranslation2d(
                    tags[i].timestamp, new Translation2d(tags[i].x, tags[i].y));
      }

      return new VisionState(estimatedPose, estimatedTags);
    } else {
      return new VisionState(estimatedPose);
    }
  }
}
