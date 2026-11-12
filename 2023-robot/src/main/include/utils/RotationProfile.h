#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/trajectory/Trajectory.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/time.h>

#include <vector>

#define TRAJECTORY_BASED_PROFILE

class RotationProfile {
 public:
  /**
   * Represents a state of the profile at a point in time.
   */
  struct State {
    /**
     * Timestamp.
     */
    units::second_t time;

    /**
     * Rotation.
     */
    frc::Rotation2d rotation;

    /**
     * Angular velocity.
     */
    units::radians_per_second_t velocity = 0_rad_per_s;
  };

  RotationProfile() = default;

  /**
   * Creates a holonomic rotation profile to smoothly
   * rotate the robot between two orientation within
   * the specified time frame and subject to maximum
   * angular acceleration and velocity constraints.
   */
  RotationProfile(
    const frc::Rotation2d& start,
    const frc::Rotation2d& end,
    const units::second_t timeframe,
    const units::radians_per_second_t maxVelocity,
    const units::radians_per_second_squared_t maxAcceleration) :
    RotationProfile(start, end, timeframe, maxVelocity, maxAcceleration, maxAcceleration) {}

  /**
   * Creates a holonomic rotation profile to smoothly
   * rotate the robot between two orientation within
   * the specified time frame and subject to maximum
   * angular acceleration and velocity constraints.
   */
  RotationProfile(
    const frc::Rotation2d& start,
    const frc::Rotation2d& end,
    const units::second_t timeframe,
    const units::radians_per_second_t maxVelocity,
    const units::radians_per_second_squared_t maxAcceleration,
    const units::radians_per_second_squared_t maxDeceleration);

  /**
   * Samples the profile at a point in time and returns the state.
   */
  State Sample(const units::second_t t) const;

  /**
   * Returns the states of the profile.
   */
  const std::vector<State> States() const;

  /**
   * Returns initial rotation.
   */
  frc::Rotation2d InitialRotation() const {
    return States().empty() ? frc::Rotation2d(m_offset) : States().front().rotation;
  }

  /**
   * Returns final rotation.
   */
  frc::Rotation2d FinalRotation() const {
    return States().empty() ? frc::Rotation2d(m_offset) : States().back().rotation;
  }

 private:
  std::vector<State> m_states;
  frc::Trajectory m_trajectory;
  units::radian_t m_offset;
  double m_sign = 1.0;

  State convert(const frc::Trajectory::State& state) const;
};
