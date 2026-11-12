package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.lib.ObjectDetectionPacket;
import frc.robot.lib.TimestampedTranslation2d;
import frc.robot.lib.UdpListenerThread;
import java.util.concurrent.atomic.AtomicLong;

public class ObjectDetector {
  private final UdpListenerThread<ObjectDetectionPacket> listener;
  private AtomicLong latency = new AtomicLong(0);

  private static long kTimeSyncCutoffMicroseconds = 2000000; // 2 seconds

  private static final TimestampedTranslation2d noObject =
      new TimestampedTranslation2d(-1, Translation2d.kZero);

  private TimestampedTranslation2d detectedObject = noObject;

  public ObjectDetector() {
    listener =
        new UdpListenerThread<ObjectDetectionPacket>(
            Constants.Network.UDP_PORT_OBJECT_DETECTOR,
            1500,
            this::packetReceived,
            ObjectDetectionPacket.struct::unpack);

    listener.start();
  }

  private void packetReceived(ObjectDetectionPacket packet) {
    if (packet.count > 0) {
      final var t = RobotController.getFPGATime();

      // ignore packets that are too far into the past or into the future
      // indicating time sync issues
      if (Math.abs(t - packet.timestamp) > kTimeSyncCutoffMicroseconds) {
        return;
      }

      latency.set(t - packet.timestamp);

      // TODO: decide on which object(s) to take when multiple are detected
      final var o = packet.objects[0];
      final var translation =
          new Translation2d(o.distance, 0).rotateBy(Rotation2d.fromRadians(o.angle));

      synchronized (this) {
        detectedObject = new TimestampedTranslation2d(packet.timestamp, translation);
      }
    }
  }

  /** Returns recent latency in milliseconds. */
  @Logged(name = "LatencyMS", importance = Importance.DEBUG)
  public double getLatency() {
    return latency.get() / 1000.0;
  }

  /** Returns most recently detected object. */
  public TimestampedTranslation2d getDetectedObject() {
    synchronized (this) {
      return detectedObject;
    }
  }
}
