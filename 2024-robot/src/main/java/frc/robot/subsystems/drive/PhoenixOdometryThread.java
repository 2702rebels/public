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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import frc.robot.Constants;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class PhoenixOdometryThread extends Thread {
  private final DriveOdometryTracker tracker;

  // prevents conflicts when registering signals
  private final Lock signalsLock = new ReentrantLock();
  private BaseStatusSignal[] signals = new BaseStatusSignal[0];
  private boolean isCANFD = false;

  private static PhoenixOdometryThread instance = null;

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    tracker = DriveOdometryTracker.getInstance();
    setName("PhoenixOdometryThread");
    setDaemon(true);
    start();
  }

  public Queue<Double> registerSignal(ParentDevice device, StatusSignal<Double> signal) {
    isCANFD = CANBus.isNetworkFD(device.getNetwork());

    signalsLock.lock();
    try {
      // create new signal placeholder
      final var newSignals = new BaseStatusSignal[signals.length + 1];
      System.arraycopy(signals, 0, newSignals, 0, signals.length);
      newSignals[signals.length] = signal;

      return tracker.registerSignal(() -> OptionalDouble.of(signal.getValueAsDouble()));
    } finally {
      signalsLock.unlock();
    }
  }

  @Override
  public void run() {
    while (true) {
      // wait for updates from all signals
      signalsLock.lock();
      try {
        if (isCANFD) {
          BaseStatusSignal.waitForAll(2.0 / Constants.odometryFrequency, signals);
        } else {
          // "waitForAll" does not support blocking on multiple
          // signals with a bus that is not CAN FD, regardless
          // of Pro licensing; no reasoning for this behavior
          // is provided by the documentation
          Thread.sleep((long) (1000.0 / Constants.odometryFrequency));
          if (signals.length > 0) BaseStatusSignal.refreshAll(signals);
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      } finally {
        signalsLock.unlock();
      }

      // save new data to queues
      tracker.update();
    }
  }
}
