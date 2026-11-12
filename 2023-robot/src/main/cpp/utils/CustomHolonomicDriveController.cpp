#include "utils/CustomHolonomicDriveController.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/math.h>

#include "Directives.h"

CustomHolonomicDriveController::CustomHolonomicDriveController(
  frc2::PIDController xController,
  frc2::PIDController yController,
  frc2::PIDController thetaController) :
  m_xController(std::move(xController)),
  m_yController(std::move(yController)),
  m_thetaController(std::move(thetaController)) {
  m_thetaController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
}

bool CustomHolonomicDriveController::AtGoal() const {
  return m_xController.AtSetpoint() && m_yController.AtSetpoint() && m_thetaController.AtSetpoint();
}

frc::ChassisSpeeds CustomHolonomicDriveController::Calculate(
  const frc::Pose2d& currentPose,
  const frc::Pose2d& trajectoryPose,
  const units::meters_per_second_t desiredLinearVelocity,
  const frc::Rotation2d& desiredRotation,
  const units::radians_per_second_t desiredAngularVelocity) {
  // calculate feed-forward velocities (field-relative)
  const auto xFF = desiredLinearVelocity * trajectoryPose.Rotation().Cos();
  const auto yFF = desiredLinearVelocity * trajectoryPose.Rotation().Sin();
  const auto thetaFF = desiredAngularVelocity;

  if (!m_enabled) {
    return frc::ChassisSpeeds::FromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.Rotation());
  }

  // calculate feedback velocities (based on position error)
  const auto xFeedback = units::meters_per_second_t {
    m_xController.Calculate(currentPose.X().value(), trajectoryPose.X().value()),
  };

  const auto yFeedback = units::meters_per_second_t {
    m_yController.Calculate(currentPose.Y().value(), trajectoryPose.Y().value()),
  };

  const auto thetaFeedback = units::radians_per_second_t {
    m_thetaController.Calculate(
      currentPose.Rotation().Radians().value(), desiredRotation.Radians().value()),
  };

#ifdef LOG_AUTO_COMMAND
  // clang-format off
  frc::SmartDashboard::PutNumber("Telemetry/Holonomic/ΔΘ ", units::convert<units::radians, units::degrees>(m_thetaController.GetPositionError()));
  frc::SmartDashboard::PutNumber("Telemetry/Holonomic/ΔX", m_xController.GetPositionError());
  frc::SmartDashboard::PutNumber("Telemetry/Holonomic/ΔY", m_yController.GetPositionError());
  frc::SmartDashboard::PutNumber("Telemetry/Holonomic/Θ FF", thetaFF.convert<units::degrees_per_second>().value());
  frc::SmartDashboard::PutNumber("Telemetry/Holonomic/Θ FB", thetaFeedback.convert<units::degrees_per_second>().value());
  frc::SmartDashboard::PutNumber("Telemetry/Holonomic/X FF", xFF.convert<units::meters_per_second>().value());
  frc::SmartDashboard::PutNumber("Telemetry/Holonomic/X FB", xFeedback.convert<units::meters_per_second>().value());
  frc::SmartDashboard::PutNumber("Telemetry/Holonomic/Y FF", yFF.convert<units::meters_per_second>().value());
  frc::SmartDashboard::PutNumber("Telemetry/Holonomic/Y FB", yFeedback.convert<units::meters_per_second>().value());
  // clang-format on
#endif

  return frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    xFF + xFeedback, yFF + yFeedback, thetaFF + thetaFeedback, currentPose.Rotation());
}

frc::ChassisSpeeds CustomHolonomicDriveController::Calculate(
  const frc::Pose2d& currentPose,
  const frc::Trajectory::State& desiredState,
  const RotationProfile::State& desiredRotation) {
  return Calculate(
    currentPose,
    desiredState.pose,
    desiredState.velocity,
    desiredRotation.rotation,
    desiredRotation.velocity);
}
