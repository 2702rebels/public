#include "utils/MathUtils.h"

#include <cmath>
#include <numbers>

constexpr static auto onePi = units::radian_t { std::numbers::pi };
constexpr static auto twoPi = units::radian_t { 2 * std::numbers::pi };

double MathUtils::RoundToStep(double value, double step) {
  return step > 0 ? floor(value / step) * step : value;
}

bool MathUtils::IsZero(double value, double epsilon) {
  return fabs(value) < epsilon;
}

units::radian_t MathUtils::ClampAngle(units::radian_t value) {
  while (value <= -onePi) {
    value += twoPi;
  }

  while (value > onePi) {
    value -= twoPi;
  }

  return value;
}

units::radian_t MathUtils::ShortestArc(units::radian_t from, units::radian_t to) {
  // the sign of delta indicates the direction of the rotation
  // positive - CCW, negative - CW
  //
  // for example: from -> to = delta
  // 30° -> -30° = -60°
  // 30° -> 150° = 120°
  // 30° -> 300° = -90°
  // -150° -> 150° = -60°
  // -30° -> -270° = 120°
  return ClampAngle(to - from);
}
