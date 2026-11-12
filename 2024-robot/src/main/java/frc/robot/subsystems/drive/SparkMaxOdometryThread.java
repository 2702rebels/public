// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.function.Supplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkMax that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public class SparkMaxOdometryThread {
  private final DriveOdometryTracker tracker;
  private final Notifier notifier;

  private static SparkMaxOdometryThread instance = null;

  public static SparkMaxOdometryThread getInstance() {
    if (instance == null) {
      instance = new SparkMaxOdometryThread();
    }
    return instance;
  }

  private SparkMaxOdometryThread() {
    tracker = DriveOdometryTracker.getInstance();
    notifier = new Notifier(this::periodic);
    notifier.setName("SparkMaxOdometryThread");
    notifier.startPeriodic(1.0 / Constants.odometryFrequency);
  }

  public Queue<Double> registerSignal(Supplier<OptionalDouble> signal) {
    return tracker.registerSignal(signal);
  }

  private void periodic() {
    tracker.update();
  }
}
