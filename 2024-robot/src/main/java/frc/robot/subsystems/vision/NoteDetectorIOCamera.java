package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

public class NoteDetectorIOCamera implements NoteDetectorIO {
  private static final double[] EmptyDoubleArray = new double[] {};
  private static final double[][] EmptyDoubleArrayArray = new double[][] {};
  private final DoubleArraySubscriber subscriber;

  public NoteDetectorIOCamera() {
    subscriber =
        NetworkTableInstance.getDefault()
            .getTable("Vision")
            .getDoubleArrayTopic("NoteBox")
            .subscribe(
                EmptyDoubleArray, PubSubOption.keepDuplicates(false), PubSubOption.sendAll(true));
  }

  @Override
  public void updateInputs(NoteDetectorIOInputs inputs) {
    final var queue = subscriber.readQueue();
    if (queue.length == 0) {
      inputs.timestamps = EmptyDoubleArray;
      inputs.frames = EmptyDoubleArrayArray;
    } else {
      // do not reallocate arrays if size did not change
      if (queue.length != inputs.timestamps.length) {
        inputs.timestamps = new double[queue.length];
        inputs.frames = new double[queue.length][];
      }

      for (var i = 0; i < queue.length; ++i) {
        inputs.timestamps[i] = queue[i].timestamp / 1E6;
        inputs.frames[i] = queue[i].value;
      }
    }
  }
}
