#include "utils/RotationProfile.h"

#include <frc/DataLogManager.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/math.h>

#include <algorithm>

#include "utils/MathUtils.h"

RotationProfile::RotationProfile(
  const frc::Rotation2d& start,
  const frc::Rotation2d& end,
  const units::second_t timeframe,
  const units::radians_per_second_t maxVelocity,
  const units::radians_per_second_squared_t maxAcceleration,
  const units::radians_per_second_squared_t maxDeceleration) {
  // constrain angles to (-pi, pi] range
  const auto r0 = MathUtils::ClampAngle(start.Radians());
  const auto rN = MathUtils::ClampAngle(end.Radians());

  if (units::math::fabs(r0 - rN) > 1_deg) {
    // difference between start and end orientation
    // the sign of delta indicates the direction of the rotation
    // positive - CCW, negative - CW
    const auto delta = MathUtils::ShortestArc(r0, rN);
    m_sign = delta.value() < 0 ? -1.0 : 1.0;
    m_offset = r0;

#ifdef TRAJECTORY_BASED_PROFILE
    // "abuse" trajectory generation mechanism treating angular parameters as linear
    // and using X coordinate as angular position
    frc::TrajectoryConfig config(
      units::meters_per_second_t { maxVelocity.convert<units::rad_per_s>().value() },
      units::meters_per_second_squared_t {
        maxAcceleration.convert<units::rad_per_s_sq>().value() });

    std::vector<frc::Pose2d> poses;
    poses.push_back({ 0_m, 0_m, frc::Rotation2d() });
    poses.push_back(
      { units::meter_t { fabs(delta.convert<units::radians>().value()) }, 0_m, frc::Rotation2d() });

    m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(poses, config);
#else
    // the profile is essentially trapezoidal profile that follows
    // acceleration, constant velocity, deceleration path to fit
    // within the given timeframe
    m_states.push_back({ 0_s, frc::Rotation2d(r0), 0_rad_per_s });  // start state

    auto tAcc = maxVelocity / maxAcceleration;  // time to maximum velocity
    auto tDec = maxVelocity / maxDeceleration;  // time from maximum velocity

    // do we need to accelerate to the maximum velocity (full trapezoid)
    if (tAcc + tDec < timeframe) {
      const auto r1 = MathUtils::ClampAngle(r0 + m_sign * maxAcceleration * tAcc * tAcc / 2);
      const auto r2 = MathUtils::ClampAngle(rN - m_sign * maxDeceleration * tDec * tDec / 2);
      const auto dt = units::math::fabs(r2 - r1) / maxVelocity;

      m_states.push_back({ tAcc, frc::Rotation2d(r1), m_sign * maxVelocity });
      m_states.push_back({ tAcc + dt, frc::Rotation2d(r2), m_sign * maxVelocity });
      m_states.push_back({ tAcc + dt + tDec, frc::Rotation2d(rN), 0_rad_per_s });  // end state
    } else {
      tAcc = timeframe * (maxAcceleration / (maxAcceleration + maxDeceleration));
      tDec = timeframe - tAcc;
      const auto r1 = MathUtils::ClampAngle(r0 + m_sign * maxAcceleration * tAcc * tAcc / 2);
      m_states.push_back({ tAcc, frc::Rotation2d(r1), m_sign * maxAcceleration * tAcc });
      m_states.push_back({ timeframe, frc::Rotation2d(rN), 0_rad_per_s });  // end state
    }
#endif
  } else {
    m_offset = r0;
  }
}

RotationProfile::State RotationProfile::convert(const frc::Trajectory::State& state) const {
  return {
    state.t,
    frc::Rotation2d(
      MathUtils::ClampAngle(m_offset + units::radian_t { m_sign * state.pose.X().value() })),
    units::radians_per_second_t { m_sign * state.velocity.value() },
  };
}

const std::vector<RotationProfile::State> RotationProfile::States() const {
#ifdef TRAJECTORY_BASED_PROFILE
  std::vector<RotationProfile::State> states;
  for (const auto& state : m_trajectory.States()) {
    states.push_back(convert(state));
  }
  return states;
#else
  return m_states;
#endif
}

RotationProfile::State RotationProfile::Sample(const units::second_t t) const {
#ifdef TRAJECTORY_BASED_PROFILE
  if (m_trajectory.States().empty()) {
    return RotationProfile::State { t, frc::Rotation2d(m_offset), 0_rad_per_s };
  }
  return convert(m_trajectory.Sample(t));
#else
  if (m_states.empty()) {
    return { t, frc::Rotation2d(), 0_rad_per_s };
  }

  if (t <= m_states.front().time) {
    return m_states.front();
  } else if (t >= m_states.back().time) {
    return m_states.back();
  }

  // use binary search to find the entry with a timestamp no less than the requested one;
  // start at 1 since we need previous entry for interpolation
  const auto state =
    std::lower_bound(m_states.cbegin() + 1, m_states.cend(), t, [](const auto& a, const auto& b) {
      return a.time < b;
    });

  const auto prev = state - 1;

  if (units::math::fabs(state->time - t) < 1E-3_s) {
    return *state;
  }

  // interpolate between states
  const auto dt = t - prev->time;
  const auto alpha = (t - prev->time) / (state->time - prev->time);
  const auto v = (1 - alpha) * prev->velocity + alpha * state->velocity;
  const auto r =
    MathUtils::ClampAngle(prev->rotation.Radians() + (state->velocity + prev->velocity) * dt / 2);

  return { t, frc::Rotation2d(r), v };
#endif
}
