package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/** Maintains shared queues for high-frequency odometry readings. */
public class DriveOdometryTracker {
  @AutoLog
  public static class DriveOdometryInputs {
    public double[] timestamps = new double[] {};
  }

  private static final int maxQueueCapacity = 16;

  private final Lock lock = new ReentrantLock();
  private final List<Queue<Double>> queues = new ArrayList<>();
  private final List<Supplier<OptionalDouble>> signals = new ArrayList<>();
  private final Queue<Double> timestamps = new ArrayBlockingQueue<>(maxQueueCapacity);

  private static DriveOdometryTracker instance = null;

  public static DriveOdometryTracker getInstance() {
    if (instance == null) {
      instance = new DriveOdometryTracker();
    }
    return instance;
  }

  /** Registers the specified signal and returns the corresponding queue. */
  public Queue<Double> registerSignal(Supplier<OptionalDouble> signal) {
    final Queue<Double> queue = new ArrayBlockingQueue<>(maxQueueCapacity);
    lock.lock();
    try {
      signals.add(signal);
      queues.add(queue);
    } finally {
      lock.unlock();
    }
    return queue;
  }

  /** Runs update on all the signals. */
  public void update() {
    // IMPORTANT: we are grabbing true timestamp here, i.e. directly from FPGA,
    // these are not "replayable" in AK and therefore must be recorded as inputs,
    // see code in updateInputs below
    final var timestamp = Logger.getRealTimestamp() / 1E6;
    lock.lock();
    try {
      var allSignalsPresent = true;
      final var values = new double[signals.size()];
      for (var i = 0; i < values.length; ++i) {
        final var value = signals.get(i).get();
        if (value.isPresent()) {
          values[i] = value.getAsDouble();
        } else {
          allSignalsPresent = false;
          break;
        }
      }
      if (allSignalsPresent) {
        timestamps.offer(timestamp);
        for (var i = 0; i < values.length; ++i) {
          queues.get(i).offer(values[i]);
        }
      }
    } finally {
      lock.unlock();
    }
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(DriveOdometryInputs inputs, Runnable updater) {
    lock.lock();
    try {
      // read timestamps and clear the queue
      inputs.timestamps = timestamps.stream().mapToDouble(Double::valueOf).toArray();
      timestamps.clear();

      updater.run();
    } finally {
      lock.unlock();
    }
  }
}
