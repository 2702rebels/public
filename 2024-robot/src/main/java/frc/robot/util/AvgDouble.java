// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class AvgDouble {
  private double[] values;
  private int iter = 0;
  private int valCount = 0;

  public AvgDouble(int size) {
    values = new double[size];
    this.clear();
  }

  public void add(double value) {
    values[iter++] = value;
    if (valCount < values.length) valCount = iter;
    if (iter == values.length) iter = 0;
  }

  public void clear() {
    for (int i = 0; i < values.length; i++) {
      values[i] = 0.0;
    }
    valCount = 0;
  }

  public double getAverage() {
    double avg = 0.0;
    for (int i = 0; i < valCount; i++) {
      avg += values[i];
    }
    return avg / valCount;
  }
}
