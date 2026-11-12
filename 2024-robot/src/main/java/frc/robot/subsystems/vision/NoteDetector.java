package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class NoteDetector extends SubsystemBase {
  // spotless:off
  private static final LoggedTunableNumber detectionProbabilityCutoff = new LoggedTunableNumber("Vision/NoteDetector/Cutoff P", 0.7);
  private static final LoggedTunableNumber detectionTimeCutoff = new LoggedTunableNumber("Vision/NoteDetector/Cutoff T", 1);
  // spotless:on

  private static final double[] EmptyDoubleArray = new double[] {};
  private final DoubleArraySubscriber subscriber;

  private DetectedNote trackedNote = null;
  private boolean hasTrackedNote = false;

  public NoteDetector() {
    subscriber =
        NetworkTableInstance.getDefault()
            .getTable("Vision")
            .getDoubleArrayTopic("NoteBox")
            .subscribe(
                EmptyDoubleArray, PubSubOption.keepDuplicates(false), PubSubOption.sendAll(true));
  }

  private static Rotation2d getYaw(double x) {
    // normalize X from camera frame pixels to [-1,1] range
    final var nx = 2 * (x / NoteDetectorConstants.FrameWidthPx) - 1;
    return Rotation2d.fromRadians(Math.atan(nx * NoteDetectorConstants.YawFactor));
  }

  private static double getDistanceMeters(double w) {
    final var d = NoteDetectorConstants.NoteWidthMeters * NoteDetectorConstants.FocalLengthPx / w;
    return Math.sqrt(d * d - NoteDetectorConstants.CameraHeightMetersSq);
  }

  public void periodic() {
    final var localTimestamp = Timer.getFPGATimestamp();
    var visionTimestamp = 0.0;

    final var cutoff = detectionProbabilityCutoff.get();
    final var queue = subscriber.readQueue();
    if (queue.length > 0) {
      for (var i = 0; i < queue.length; ++i) {
        final var timestamp = queue[i].timestamp / 1E6;
        final var frame = queue[i].value;
        visionTimestamp = timestamp;

        // keep track of the note detected with highest probability
        // if multiple notes are detected within the frame
        DetectedNote bestNote = null;

        // array contains consecutive 6 tuples of the following format
        // [type, probability, xmin, ymin, xmax, ymax]
        // type = 0 -- note
        // type = 1 -- robot
        var offset = 0;
        while (offset + 6 <= frame.length) {
          final var type = frame[offset];
          if (type != 0) {
            offset += 6;
            continue;
          }

          final var p = frame[offset + 1];

          // ignore observations with low detection probabilities
          if (p > cutoff) {
            final var xmin = frame[offset + 2];
            final var xmax = frame[offset + 4];
            // final var ymin = frame[offset + 3];
            // final var ymax = frame[offset + 5];

            final var w = Math.abs(xmax - xmin);

            // guard against garbage values
            if (w > 0) {
              final var yaw = getYaw((xmax + xmin) * 0.5);
              final var d = getDistanceMeters(w);

              // if bbox is "touching" edge of the frame, then we cannot guarantee
              // that entire note is within FOV, making distance estimation unreliable
              // as it depends on the full width of the note being visible
              final var distanceTrusted = xmin > 5 && xmax < NoteDetectorConstants.FrameWidthPx - 5;

              final var note = new DetectedNote(timestamp, p, yaw, d, distanceTrusted);

              if (bestNote == null || bestNote.probability < note.probability) {
                bestNote = note;
              }
            }
          }

          offset += 6;
        }

        if (bestNote != null) {
          trackedNote = bestNote;
        }
      }
    }

    Logger.recordOutput("Vision/NoteDetector/Queue Length", queue.length);
    if (queue.length > 0) {
      Logger.recordOutput("Vision/NoteDetector/Timestamp", visionTimestamp);
      Logger.recordOutput("Vision/NoteDetector/Delay", localTimestamp - visionTimestamp);
    }

    if (trackedNote != null) {
      hasTrackedNote = localTimestamp - trackedNote.timestamp <= detectionTimeCutoff.get();
    }
  }

  public record DetectedNote(
      double timestamp,
      double probability,
      Rotation2d yaw,
      double distanceMeters,
      boolean distanceTrusted) {
    public double distanceInches() {
      return Units.metersToInches(distanceMeters);
    }
  }

  /** Returns most recently tracked note. */
  public DetectedNote getTrackedNote() {
    return hasTrackedNote ? trackedNote : null;
  }

  /** Returns an indicator that the tracked note is trusted. */
  @AutoLogOutput(key = "Vision/NoteDetector/HasTrackedNote")
  public boolean hasTrackedNote() {
    return hasTrackedNote;
  }

  /** Returns most recently detected note distance in inches. */
  @AutoLogOutput(key = "Vision/NoteDetector/NoteDistance")
  public double getNoteDistanceInches() {
    return trackedNote == null ? Double.NaN : trackedNote.distanceInches();
  }

  /** Returns most recently detected note distance trust. */
  @AutoLogOutput(key = "Vision/NoteDetector/NoteDistanceTrusted")
  public boolean getNoteDistanceTrusted() {
    return trackedNote == null ? false : trackedNote.distanceTrusted();
  }

  /** Returns most recently detected note yaw in degrees. */
  @AutoLogOutput(key = "Vision/NoteDetector/NoteYawDegrees")
  public double getNoteYawDegrees() {
    return trackedNote == null ? Double.NaN : trackedNote.yaw().getDegrees();
  }

  @AutoLogOutput(key = "Vision/NoteDetector/NoteYawRadians")
  public double getNoteYawRadians() {
    return trackedNote == null ? Double.NaN : trackedNote.yaw().getRadians();
  }
}
