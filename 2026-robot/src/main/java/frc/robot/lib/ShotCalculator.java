/*
 * ShotCalculator.java - Newton-method SOTM fire control with drag compensation
 *
 * MIT License
 *
 * Copyright (c) 2026 FRC Team 5962 perSEVERE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
 *
 * THIS FILE HAS BEEN VERY HEAVILY MODIFIED FROM ITS ORIGINAL VERSION POSTED AT
 * https://github.com/eeveemara/frc-fire-control/blob/main/src/main/java/frc/firecontrol/ShotCalculator.java
 */

package frc.robot.lib;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/** Shoot-on-the-move fire control solver. */
public class ShotCalculator {
  /** Computed firing solution parameters. */
  public static record ShotParameters(
      Translation2d target,
      Distance distance,
      Distance virtualDistance,
      Time tof,
      double shooterVelocity,
      double shooterAngle,
      double quality,
      int iterations) {}

  // solver tuning
  private static final int kMaxIterations = 10;
  private static final int kQualityIterations = 3;
  private static final double kConvergenceTolerance = 0.001; // seconds
  private static final double kMinToF = 0.05;
  private static final double kMaxToF = 5.0;

  // threshold for solver use, below this speed use direct aiming
  private static final double kMinSpeed = 0.1; // meters/second

  public final Translation2d kOffsetXY;

  public ShotCalculator(Translation2d offsetXY) {
    kOffsetXY = offsetXY;
  }

  // linear drag coefficient
  public double kDrag = 0;

  // lookup table for tuned shot parameters
  private final ShotLUT lut = new ShotLUT();

  // solver state (reused across cycles to avoid allocation)
  private double previousToF = -1;

  private ShotLUT.ShotParameters getEntry(double distance) {
    return lut.get(distance);
  }

  // central finite difference derivative of the TOF lookup table
  private static final double kDerivativeStepToF = 0.01; // 1cm step

  private double tofMapDerivative(double d) {
    final var hi = getEntry(d + kDerivativeStepToF).tof();
    final var lo = getEntry(d - kDerivativeStepToF).tof();
    return (hi - lo) / (2.0 * kDerivativeStepToF);
  }

  private static record SolverResult(double distance, double tof, double quality, int iterations) {}

  /** Newton-method solver implementation. */
  private SolverResult solve(double distance, double tx, double ty, double vx, double vy) {
    final var initialToF = getEntry(distance).tof();

    // warm start from previous cycle's solution when available
    var tof = previousToF > 0 ? previousToF : initialToF;
    var d = distance;

    var iterations = 0;
    for (int i = 0; i < kMaxIterations; ++i) {
      var prevToF = tof;

      var dragExp = 1.0;
      var effectiveToF = tof;

      // apply linear drag calculations if enabled
      if (kDrag > 1e-6) {
        dragExp = Math.exp(-kDrag * tof);
        effectiveToF = (1.0 - dragExp) / kDrag;
      }

      // projected displacement with (optionally) drag-compensated velocity offset
      final var dx = tx - vx * effectiveToF;
      final var dy = ty - vy * effectiveToF;
      d = Math.hypot(dx, dy);

      // degenerate guard: ball is essentially at the target
      if (d < 0.01) {
        tof = initialToF;
        iterations = kMaxIterations + 1; // mark as diverged
        break;
      }

      final var lookupToF = getEntry(d).tof();
      final var e = tof - lookupToF;

      // Newton-step derivative
      // if we are using linear drag compensation then D' should be multiplied by exp(-kt)
      final var dPrime = -dragExp * (dx * vx + dy * vy) / d;
      final var ePrime = 1.0 - tofMapDerivative(d) * dPrime;

      // Newton-step with near-zero denominator guard
      if (Math.abs(ePrime) > 0.01) {
        tof -= e / ePrime;
      } else {
        tof = lookupToF; // fixed-point fallback
      }

      // per-iteration clamp to prevent runaway
      tof = MathUtil.clamp(tof, kMinToF, kMaxToF);
      iterations = i + 1;

      // convergence check
      if (Math.abs(tof - prevToF) < kConvergenceTolerance) {
        break;
      }
    }

    // divergence guard
    if (tof > kMaxToF || tof < 0.0 || Double.isNaN(tof)) {
      tof = initialToF;
      iterations = kMaxIterations + 1; // mark as diverged
    }

    // estimate solution quality [0-1] based on convergence
    final var quality =
        iterations > kMaxIterations
            ? 0.0 // diverged
            : iterations <= kQualityIterations
                ? 1.0
                : MathUtil.interpolate(
                    1.0,
                    0.1,
                    (double) (iterations - kQualityIterations)
                        / (kMaxIterations - kQualityIterations));

    return new SolverResult(d, tof, quality, iterations);
  }

  /** Solves for the firing solution based on the current robot pose and field-relative velocity. */
  public ShotParameters calculate(
      Pose2d robotPose, ChassisSpeeds fieldVelocity, Translation2d target) {
    // field-relative shooter pose
    final var offsetXY = kOffsetXY.rotateBy(robotPose.getRotation());
    final var shooterXY = robotPose.getTranslation().plus(offsetXY);

    // field-relative shooter velocity accounting for omega: v + omega x r
    final var omega = fieldVelocity.omegaRadiansPerSecond;
    final var vx = fieldVelocity.vxMetersPerSecond - offsetXY.getY() * omega;
    final var vy = fieldVelocity.vyMetersPerSecond + offsetXY.getX() * omega;

    // displacement from target to the shooter
    final var d = target.minus(shooterXY);
    final var distance = d.getNorm();
    final var speed = Math.hypot(vx, vy);

    final var velocityFiltered = speed < kMinSpeed;
    final var result =
        velocityFiltered
            ? new SolverResult(distance, getEntry(distance).tof(), 1, 0)
            : solve(distance, d.getX(), d.getY(), vx, vy);

    // save for next cycle's warm start
    previousToF = result.tof;

    // aim at velocity-compensated target position
    var tx = target.getX();
    var ty = target.getY();

    if (!velocityFiltered) {
      var effectiveToF = result.tof;

      // apply linear drag compensation when calculating virtual target
      if (kDrag > 1e-6) {
        effectiveToF = (1.0 - Math.exp(-kDrag * effectiveToF)) / kDrag;
      }

      tx -= vx * effectiveToF;
      ty -= vy * effectiveToF;
    }

    final var r = getEntry(result.distance);
    return new ShotParameters(
        new Translation2d(tx, ty),
        Meters.of(distance),
        Meters.of(result.distance),
        Seconds.of(result.tof),
        r.velocity(),
        r.angle(),
        result.quality,
        result.iterations);
  }

  /** Adds a LUT entry. */
  public void putLUTEntry(Distance distance, double velocity, double angle, double tof) {
    lut.put(distance.in(Meters), new ShotLUT.ShotParameters(velocity, angle, tof));
  }

  /** Gets interpolated LUT entry. */
  public ShotLUT.ShotParameters getEntry(Distance distance) {
    return getEntry(distance.in(Meters));
  }

  /** Resets the warm start state (invoke after a pose reset to avoid use of stale data). */
  public void resetWarmStart() {
    previousToF = -1;
  }
}
