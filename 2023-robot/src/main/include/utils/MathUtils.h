#pragma once

#include <units/angle.h>

class MathUtils {
 public:
  /**
   * Rounds (floors) a value to the specified step size.
   * For instance, this can be used to discretize continuous inputs
   * to a specific values.
   *
   * @param value The value to round.
   * @param step The step size.
   * @return The rounded value.
   */
  static double RoundToStep(double value, double step);

  /**
   * Tests whether the specified value is "nearly" zero.
   */
  static bool IsZero(double value, double epsilon = 0.000001);

  /**
   * Clamps angle to (-pi, pi] range.
   */
  static units::radian_t ClampAngle(units::radian_t value);

  /**
   * Computes the shortest angular arc between two angular positions.
   * The sign of returned value indicates the direction of the rotation:
   * positive - CCW, negative - CW
   */
  static units::radian_t ShortestArc(units::radian_t from, units::radian_t to);
};
