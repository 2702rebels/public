package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AprilTagDetector extends SubsystemBase {
  private static final double[] EmptyDoubleArray = new double[] {};
  private final DoubleArraySubscriber subscriber;

  private final AprilTag[] tags = new AprilTag[18]; // 16 april tags locations + 2 virtual
  private final double cutoff = 1e15; // Vision sends -3e16

  public AprilTagDetector() {
    subscriber =
        NetworkTableInstance.getDefault()
            .getTable("Vision")
            .getDoubleArrayTopic("PoseArray")
            .subscribe(
                EmptyDoubleArray, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

    // initialize tags
    for (var i = 0; i < tags.length; ++i) {
      tags[i] = new AprilTag(i + 1);
    }

    SmartDashboard.putNumber("Vision/Exposure", 480);
  }

  public void periodic() {
    final var localTimestamp = Timer.getFPGATimestamp();
    var visionTimestamp = 0.0;

    final var queue = subscriber.readQueue();
    if (queue.length > 0) {
      for (var i = 0; i < queue.length; ++i) {
        final var timestamp = queue[i].timestamp / 1E6;
        final var frame = queue[i].value;
        visionTimestamp = timestamp;

        // array contains consecutive 3 tuples of the following format
        // [x, y, z]
        for (var j = 0; j < tags.length; ++j) {
          final var offset = j * 3;
          // incomplete frame (should not happen)
          if (offset + 2 >= frame.length) {
            break;
          }

          final var x = frame[offset];
          if (Math.abs(x) > cutoff) {
            // keep the last known vector and timestamp but set tracked to false
            // allows using historical observation if we have relevant odometry
            tags[j].tracked = false;
          } else {
            tags[j].tracked = true;
            tags[j].x = x;
            tags[j].y = frame[offset + 1];
            tags[j].z = frame[offset + 2];
            tags[j].timestamp = timestamp;
          }
        }
      }
    }

    Logger.recordOutput("Vision/AprilTags/Queue Length", queue.length);
    if (queue.length > 0) {
      Logger.recordOutput("Vision/AprilTags/Timestamp", visionTimestamp);
      Logger.recordOutput("Vision/AprilTags/Delay", localTimestamp - visionTimestamp);
    }

    for (var i = 0; i < tags.length; ++i) {
      final var key = "Vision/AprilTags/" + tags[i].id;
      Logger.recordOutput(key + "/Tracked", tags[i].tracked);
      Logger.recordOutput(key + "/X", tags[i].x);
      Logger.recordOutput(key + "/Y", tags[i].y);
      Logger.recordOutput(key + "/Z", tags[i].z);
    }
  }

  /**
   * Gets the tag with the specified identifier.
   *
   * @param id field-specific identifier (one-based)
   */
  public AprilTag getTag(int id) {
    return tags[id - 1];
  }

  /** Determines whether tags of interest are tracked. */
  public boolean areTagsTracked(int... tagIds) {
    for (var id : tagIds) {
      if (tags[id - 1].tracked) {
        return true;
      }
    }
    return false;
  }
}
