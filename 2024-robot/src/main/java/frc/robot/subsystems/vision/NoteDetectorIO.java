package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface NoteDetectorIO {
  public class NoteDetectorIOInputs implements LoggableInputs {
    private static final double[] EmptyDoubleArray = new double[] {};

    public double[] timestamps = new double[] {};
    public double[][] frames = new double[][] {};

    @Override
    public void toLog(LogTable table) {
      table.put("Timestamps", timestamps);
      table.put("FrameCount", frames.length);
      for (var i = 0; i < frames.length; ++i) {
        table.put("Frame/" + i, frames[i]);
      }
    }

    @Override
    public void fromLog(LogTable table) {
      timestamps = table.get("Timestamps", EmptyDoubleArray);
      final var frameCount = table.get("FrameCount", 0);
      frames = new double[frameCount][];
      for (var i = 0; i < frameCount; ++i) {
        frames[i] = table.get("Frame/" + i, EmptyDoubleArray);
      }
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(NoteDetectorIOInputs inputs) {}
}
