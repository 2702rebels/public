#include "commands/DriveRotateCommand.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <units/math.h>

#include "Settings.h"
#include "utils/MathUtils.h"

// clang-format off
REGISTER_DOUBLE(AutoDriveRotateHP,   "Auto/DriveRotate/Theta - P", AutoConstants::kPThetaController);
REGISTER_DOUBLE(AutoDriveRotateHI,   "Auto/DriveRotate/Theta - I", AutoConstants::kIThetaController);
REGISTER_DOUBLE(AutoDriveRotateHD,   "Auto/DriveRotate/Theta - D", AutoConstants::kDThetaController);
REGISTER_DOUBLE(AutoDriveRotateHV,   "Auto/DriveRotate/Theta - v", AutoConstants::kMaxAngularSpeed.value());
REGISTER_DOUBLE(AutoDriveRotateHA,   "Auto/DriveRotate/Theta - a", AutoConstants::kMaxAngularAcceleration.value());
REGISTER_DOUBLE(AutoDriveRotateHEp,  "Auto/DriveRotate/Theta - Ep (deg)", 2);
REGISTER_DOUBLE(AutoDriveRotateHEv,  "Auto/DriveRotate/Theta - Ev (dps)", 10);

// clang-format on

DriveRotateCommand::DriveRotateCommand(
  std::function<frc::Rotation2d()> desiredRotation, DriveSubsystem* drive) :
  m_drive { drive },
  m_desiredRotation { desiredRotation },
  m_thetaController {
    frc::ProfiledPIDController<units::radians> {
      GetAutoDriveRotateHP(),
      GetAutoDriveRotateHI(),
      GetAutoDriveRotateHD(),
      frc::TrapezoidProfile<units::radians>::Constraints {
        units::degrees_per_second_t { GetAutoDriveRotateHV() },
        units::degrees_per_second_squared_t { GetAutoDriveRotateHA() },
      },
    },
  } {
  AddRequirements(drive);
  m_thetaController.EnableContinuousInput(
    units::radian_t { -std::numbers::pi }, units::radian_t { std::numbers::pi });
}

void DriveRotateCommand::Initialize() {
  m_thetaController.SetP(GetAutoDriveRotateHP());
  m_thetaController.SetI(GetAutoDriveRotateHI());
  m_thetaController.SetD(GetAutoDriveRotateHD());
  m_thetaController.SetTolerance(
    units::degree_t { GetAutoDriveRotateHEp() },
    units::degrees_per_second_t { GetAutoDriveRotateHEv() });

  m_thetaController.SetConstraints(frc::TrapezoidProfile<units::radians>::Constraints {
    units::degrees_per_second_t { GetAutoDriveRotateHV() },
    units::degrees_per_second_squared_t { GetAutoDriveRotateHA() },
  });

  m_thetaController.Reset(m_drive->GetHeading().Radians());
}

void DriveRotateCommand::Execute() {
  m_running = true;

  const auto currentRotation = m_drive->GetHeading();
  const auto desiredRotation = m_desiredRotation();

  auto thetaFF = units::radians_per_second_t { m_thetaController.Calculate(
    currentRotation.Radians(), desiredRotation.Radians()) };

  const auto thetaAtGoal = m_thetaController.AtGoal();
  if (thetaAtGoal) {
    thetaFF = 0_rad_per_s;
  }

  const auto speeds =
    frc::ChassisSpeeds::FromFieldRelativeSpeeds(0_mps, 0_mps, thetaFF, currentRotation.Radians());

  m_drive->SetChassisSpeeds(speeds);

#ifdef LOG_AUTO_COMMAND
  // clang-format off
  frc::SmartDashboard::PutNumber("Telemetry/DriveRotate/Theta Error", m_thetaController.GetPositionError().convert<units::degrees>().value());
  frc::SmartDashboard::PutBoolean("Telemetry/DriveRotate/Theta At Goal", thetaAtGoal);
  // clang-format on
#endif
}

void DriveRotateCommand::End(bool interrupted) {
  m_running = false;
  m_drive->Stop();
}

bool DriveRotateCommand::AtGoal() const {
  return m_running && m_thetaController.AtGoal();
}

bool DriveRotateCommand::AtGoal(units::radian_t rotationTolerance) const {
  return m_running && units::math::fabs(m_thetaController.GetPositionError()) < rotationTolerance;
}

bool DriveRotateCommand::IsFinished() {
  return AtGoal();
}
