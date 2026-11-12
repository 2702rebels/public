package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.NoteDetector;
import frc.robot.subsystems.vision.NoteDetectorConstants;
import frc.robot.util.Misc;

public class DriveToNote extends DriveToPose {
  private static final double noteYawTolerance = Units.degreesToRadians(1);
  private static final double noteDistanceTolerance = 0.01; // meters
  private final NoteDetector detector;
  private final Drive drive;
  private double poseTimestamp;
  private Pose2d pose;
  private double lastNoteDistance;
  private double lastNoteYaw;
  private boolean disableMovement;

  public DriveToNote(Drive drive, NoteDetector detector, boolean disableMovement) {
    super(drive, new Pose2d());
    this.detector = detector;
    this.drive = drive;
    this.disableMovement = disableMovement;
    this.poseSupplier = this::getTargetPose;
  }

  private Pose2d getTargetPose() {
    final var note = detector.getTrackedNote();
    final var currentPose = drive.getPose();

    // if we are no longer tracking any note then return current pose
    if (note == null) {
      return currentPose;
    }

    // there is a delay at receiving note data, hence we have to remember
    // the target pose based on the last received position of the note and
    // keep it until new data arrives
    //
    // additionally we should not recompute the pose if the note's perceived
    // location did not change, this happens when the camera still sends us
    // updates, but they do not represent real note position yet
    final var timestamp = note.timestamp();

    if (poseTimestamp < timestamp) {
      final var noteYaw = note.yaw().getRadians();
      final var noteDistance = note.distanceMeters();
      final var noteDistanceTrusted = note.distanceTrusted();

      final var mustDerivePose =
          pose == null
              || Math.abs(lastNoteYaw - noteYaw) >= noteYawTolerance
              || noteDistanceTrusted
                  && Math.abs(lastNoteDistance - noteDistance) >= noteDistanceTolerance;

      if (!mustDerivePose) {
        return pose;
      }

      // determine translation vector with respect to the COR;
      // except when the distance cannot be trusted, then only
      // command robot to rotate in place
      if (!disableMovement && noteDistanceTrusted) {
        final var desiredRotation = Misc.subtract(currentPose.getRotation(), note.yaw());
        final var vector =
            new Translation2d(noteDistance, 0)
                .rotateBy(desiredRotation)
                .plus(NoteDetectorConstants.CameraTranslationMeters);
        pose = new Pose2d(currentPose.getTranslation().plus(vector), desiredRotation);
      } else {
        final var desiredRotation = Misc.subtract(currentPose.getRotation(), note.yaw());
        pose = new Pose2d(currentPose.getTranslation(), desiredRotation);
      }

      lastNoteYaw = noteYaw;
      lastNoteDistance = noteDistanceTrusted ? noteDistance : lastNoteDistance;

      poseTimestamp = timestamp;
    }

    return pose;
  }
}
