#include "utils/SwerveUtils.h"

#include <cmath>
#include <numbers>

double SwerveUtils::StepTowards(double current, double target, double stepsize) {
  if (abs(current - target) <= stepsize) {
    return target;
  } else if (target < current) {
    return current - stepsize;
  } else {
    return current + stepsize;
  }
}

double SwerveUtils::StepTowardsCircular(double current, double target, double stepsize) {
  current = WrapAngle(current);
  target = WrapAngle(target);

  const auto temp = target - current;
  const auto stepDirection = temp > 0 ? 1 : temp < 0 ? -1 : 0;  // get sign
  const double difference = abs(current - target);

  if (difference <= stepsize) {
    return target;
  } else if (difference > std::numbers::pi) {
    // does the system need to wrap
    // over eventually?
    // handle the special case where you can reach the target in one step while
    // also wrapping
    if (
      current + 2 * std::numbers::pi - target < stepsize ||
      target + 2 * std::numbers::pi - current < stepsize) {
      return target;
    } else {
      return WrapAngle(current - stepDirection * stepsize);  // this will handle wrapping gracefully
    }

  } else {
    return current + stepDirection * stepsize;
  }
}

double SwerveUtils::AngleDifference(double angleA, double angleB) {
  const double difference = abs(angleA - angleB);
  return difference > std::numbers::pi ? (2 * std::numbers::pi) - difference : difference;
}

double SwerveUtils::WrapAngle(double angle) {
  constexpr double twoPi = 2 * std::numbers::pi;

  if (angle == twoPi) {
    // handle this case separately to avoid floating point errors with the floor
    // after the division in the case below
    return 0.0;
  } else if (angle > twoPi) {
    return angle - twoPi * floor(angle / twoPi);
  } else if (angle < 0.0) {
    return angle + twoPi * (floor((-angle) / twoPi) + 1);
  } else {
    return angle;
  }
}
