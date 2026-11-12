#pragma once

#include "utils/SwerveDriveKinematics2.h"

#include <frc/EigenCore.h>
#include <units/acceleration.h>

#include <Eigen/QR>

template <class... Wheels>
SwerveDriveKinematics2(frc::Translation2d, Wheels...)
  -> SwerveDriveKinematics2<1 + sizeof...(Wheels)>;

template <size_t NumModules>
wpi::array<SwerveModuleState2, NumModules>
SwerveDriveKinematics2<NumModules>::ToSwerveModuleStates2(
  const frc::ChassisSpeeds& chassisSpeeds, const frc::Translation2d& centerOfRotation) const {
  const auto moduleStates =
    frc::SwerveDriveKinematics<NumModules>::ToSwerveModuleStates(chassisSpeeds, centerOfRotation);

  if (
    chassisSpeeds.vx == 0_mps && chassisSpeeds.vy == 0_mps && chassisSpeeds.omega == 0_rad_per_s) {
    for (size_t i = 0; i < NumModules; i++) {
      m_moduleStates[i].speed = 0_mps;
      m_moduleStates[i].omega = 0_rad_per_s;
    }

    return m_moduleStates;
  }

  // we have a new center of rotation; we need to compute the matrix again
  if (centerOfRotation != m_previousCoR) {
    for (size_t i = 0; i < NumModules; i++) {
      // clang-format off
      m_inverseKinematics.template block<2, 4>(i * 2, 0) =
        frc::Matrixd<2, 4> {
          { 1, 0, (-m_modules[i].X() + centerOfRotation.X()).value(), (-m_modules[i].Y() + centerOfRotation.Y()).value() },
          { 0, 1, (-m_modules[i].Y() + centerOfRotation.Y()).value(), (+m_modules[i].X() - centerOfRotation.X()).value() }
        };
      // clang-format on
    }
    m_previousCoR = centerOfRotation;
  }

  const frc::Matrixd<4, 1> accelerationVector { 0, 0, std::pow(chassisSpeeds.omega.value(), 2), 0 };
  const frc::Matrixd<NumModules * 2, 1> accelerationStateMatrix =
    m_inverseKinematics * accelerationVector;

  for (size_t i = 0; i < NumModules; i++) {
    const units::meters_per_second_squared_t ax { accelerationStateMatrix(i * 2, 0) };
    const units::meters_per_second_squared_t ay { accelerationStateMatrix(i * 2 + 1, 0) };

    const units::meters_per_second_t speed = moduleStates[i].speed;
    const frc::Rotation2d& rotation = moduleStates[i].angle;

    // left here for reference, since we do not need linear acceleration output
    //
    // frc::Matrixd<2, 2> trigTheta = {
    //   { rotation.Cos(), rotation.Sin() },
    //   { -rotation.Sin(), rotation.Cos() },
    // };
    // const frc::Matrixd<2, 1> av { ax.value(), ay.value() };
    // const frc::Matrixd<2, 1> result = trigTheta * av;

    const auto omegaRaw =
      (-rotation.Sin() * ax.value() + rotation.Cos() * ay.value()) / speed.value();

    const auto omega = units::radians_per_second_t { omegaRaw } - chassisSpeeds.omega;

    m_moduleStates[i] = { speed, rotation, omega };
  }

  return m_moduleStates;
}

template <size_t NumModules>
void SwerveDriveKinematics2<NumModules>::DesaturateWheelSpeeds(
  wpi::array<SwerveModuleState2, NumModules>* moduleStates,
  units::meters_per_second_t attainableMaxSpeed) {
  auto& states = *moduleStates;
  auto realMaxSpeed =
    std::max_element(states.begin(), states.end(), [](const auto& a, const auto& b) {
      return units::math::abs(a.speed) < units::math::abs(b.speed);
    })->speed;

  if (realMaxSpeed > attainableMaxSpeed) {
    for (auto& module : states) {
      module.speed = module.speed / realMaxSpeed * attainableMaxSpeed;
    }
  }
}
