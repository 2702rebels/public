#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <units/angular_velocity.h>

struct SwerveModuleState2 : public frc::SwerveModuleState {
  /**
   * Angular velocity of the module.
   */
  units::radians_per_second_t omega = 0_rad_per_s;

  bool operator==(const SwerveModuleState2& other) const {
    return frc::SwerveModuleState::operator==(other) &&
           units::math::abs(omega - other.omega) < 1e-9_rad_per_s;
  }
};
