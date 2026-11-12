#pragma once

#include <units/angular_velocity.h>
#include <units/math.h>
#include <wpimath/MathShared.h>

#include <algorithm>

#include "utils/MathUtils.h"

class AngleSlewRateLimiter {
 public:
  AngleSlewRateLimiter(
    units::radians_per_second_t rateLimit, units::radian_t initialValue = { 0_rad }) :
    m_rateLimit { rateLimit },
    m_prevVal { initialValue },
    m_prevTime { wpi::math::MathSharedStore::GetTimestamp() } {}

  units::radian_t Calculate(units::radian_t input, double factor = 1.0) {
    const auto currentTime = wpi::math::MathSharedStore::GetTimestamp();
    const auto elapsedTime = currentTime - m_prevTime;
    const auto delta = MathUtils::ShortestArc(m_prevVal, input);

    // factor here allows adjusting effective rate at runtime
    const auto step = m_rateLimit * elapsedTime * factor;

    if (MathUtils::IsZero(delta.value(), step.value())) {
      m_prevVal = MathUtils::ClampAngle(input);
    } else {
      m_prevVal = MathUtils::ClampAngle(m_prevVal + std::clamp(delta, -step, step));
    }

    m_prevTime = currentTime;
    return m_prevVal;
  }

  /**
   * Instantly resets the slew rate limiter to the specified value.
   */
  void Reset(units::radian_t value) {
    m_prevVal = MathUtils::ClampAngle(value);
    m_prevTime = wpi::math::MathSharedStore::GetTimestamp();
  }

  /**
   * Sets new rate limits.
   */
  void SetLimit(units::radians_per_second_t rateLimit) {
    m_rateLimit = rateLimit;
  }

 private:
  units::radians_per_second_t m_rateLimit;
  units::radian_t m_prevVal;
  units::second_t m_prevTime;
};
