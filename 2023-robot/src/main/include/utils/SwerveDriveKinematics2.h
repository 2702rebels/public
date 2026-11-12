#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#include "utils/SwerveModuleState2.h"

template <size_t NumModules>
class SwerveDriveKinematics2 : public frc::SwerveDriveKinematics<NumModules> {
 public:
  template <typename... Wheels>
  explicit SwerveDriveKinematics2(frc::Translation2d wheel, Wheels&&... wheels) :
    frc::SwerveDriveKinematics<NumModules>(wheel, wheels...),
    m_modules { wheel, wheels... },
    m_moduleStates { wpi::empty_array } {
    for (size_t i = 0; i < NumModules; i++) {
      // clang-format off
        m_inverseKinematics.template block<2, 4>(i * 2, 0) <<
          1, 0, (-m_modules[i].X()).value(), (-m_modules[i].Y()).value(),
          0, 1, (-m_modules[i].Y()).value(), (+m_modules[i].X()).value();
      // clang-format on
    }
  }

  explicit SwerveDriveKinematics2(const wpi::array<frc::Translation2d, NumModules>& wheels) :
    frc::SwerveDriveKinematics<NumModules>(wheels),
    m_modules { wheels },
    m_moduleStates { wpi::empty_array } {
    for (size_t i = 0; i < NumModules; i++) {
      // clang-format off
        m_inverseKinematics.template block<2, 4>(i * 2, 0) <<
          1, 0, (-m_modules[i].X()).value(), (-m_modules[i].Y()).value(),
          0, 1, (-m_modules[i].Y()).value(), (+m_modules[i].X()).value();
      // clang-format on
    }
  }

  wpi::array<SwerveModuleState2, NumModules> ToSwerveModuleStates2(
    const frc::ChassisSpeeds& chassisSpeeds,
    const frc::Translation2d& centerOfRotation = frc::Translation2d {}) const;

  void DesaturateWheelSpeeds(
    wpi::array<SwerveModuleState2, NumModules>* moduleStates,
    units::meters_per_second_t attainableMaxSpeed);

 private:
  wpi::array<frc::Translation2d, NumModules> m_modules;
  mutable wpi::array<SwerveModuleState2, NumModules> m_moduleStates;
  mutable frc::Matrixd<NumModules * 2, 4> m_inverseKinematics;
  mutable frc::Translation2d m_previousCoR;
};

#include "SwerveDriveKinematics2.inc"
