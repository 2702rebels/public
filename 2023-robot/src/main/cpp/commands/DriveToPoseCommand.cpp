#include "commands/DriveToPoseCommand.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <units/velocity.h>

#include "Settings.h"
#include "utils/MathUtils.h"

// clang-format off
REGISTER_DOUBLE(AutoDriveToPoseXYP,  "Auto/DriveToPose/XY - P", 2.0);
REGISTER_DOUBLE(AutoDriveToPoseXYI,  "Auto/DriveToPose/XY - I", 0.0);
REGISTER_DOUBLE(AutoDriveToPoseXYD,  "Auto/DriveToPose/XY - D", 0.0);
REGISTER_DOUBLE(AutoDriveToPoseXYEp, "Auto/DriveToPose/XY - Ep (m)", 0.05);
REGISTER_DOUBLE(AutoDriveToPoseXYEv, "Auto/DriveToPose/XY - Ev (mps)", 0.1);
REGISTER_DOUBLE(AutoDriveToPoseXYV,  "Auto/DriveToPose/XY - v", AutoConstants::kMaxSpeed.value());
REGISTER_DOUBLE(AutoDriveToPoseXYA,  "Auto/DriveToPose/XY - a", AutoConstants::kMaxAcceleration.value());
REGISTER_DOUBLE(AutoDriveToPoseHP,   "Auto/DriveToPose/Theta - P", AutoConstants::kPThetaController);
REGISTER_DOUBLE(AutoDriveToPoseHI,   "Auto/DriveToPose/Theta - I", AutoConstants::kIThetaController);
REGISTER_DOUBLE(AutoDriveToPoseHD,   "Auto/DriveToPose/Theta - D", AutoConstants::kDThetaController);
REGISTER_DOUBLE(AutoDriveToPoseHV,   "Auto/DriveToPose/Theta - v", AutoConstants::kMaxAngularSpeed.value());
REGISTER_DOUBLE(AutoDriveToPoseHA,   "Auto/DriveToPose/Theta - a", AutoConstants::kMaxAngularAcceleration.value());
REGISTER_DOUBLE(AutoDriveToPoseHEp,  "Auto/DriveToPose/Theta - Ep (deg)", 2);
REGISTER_DOUBLE(AutoDriveToPoseHEv,  "Auto/DriveToPose/Theta - Ev (dps)", 10);

// clang-format on

DriveToPoseCommand::DriveToPoseCommand(
  std::function<frc::Pose2d()> desiredPose, DriveSubsystem* drive, bool endAtGoal) :
  m_endAtGoal { endAtGoal },
  m_drive { drive },
  m_desiredPose { desiredPose },
  m_driveController {
    frc::ProfiledPIDController<units::meter> {
      GetAutoDriveToPoseXYP(),
      GetAutoDriveToPoseXYI(),
      GetAutoDriveToPoseXYD(),
      frc::TrapezoidProfile<units::meter>::Constraints {
        units::meters_per_second_t { GetAutoDriveToPoseXYV() },
        units::meters_per_second_squared_t { GetAutoDriveToPoseXYA() },
      },
    },
  },
  m_thetaController {
    frc::ProfiledPIDController<units::radians> {
      GetAutoDriveToPoseHP(),
      GetAutoDriveToPoseHI(),
      GetAutoDriveToPoseHD(),
      frc::TrapezoidProfile<units::radians>::Constraints {
        units::degrees_per_second_t { GetAutoDriveToPoseHV() },
        units::degrees_per_second_squared_t { GetAutoDriveToPoseHA() },
      },
    },
  } {
  AddRequirements(drive);
  m_thetaController.EnableContinuousInput(
    units::radian_t { -std::numbers::pi }, units::radian_t { std::numbers::pi });
}

void DriveToPoseCommand::Initialize() {
  m_driveController.SetP(GetAutoDriveToPoseXYP());
  m_driveController.SetI(GetAutoDriveToPoseXYI());
  m_driveController.SetD(GetAutoDriveToPoseXYD());
  m_driveController.SetTolerance(
    units::meter_t { GetAutoDriveToPoseXYEp() },
    units::meters_per_second_t { GetAutoDriveToPoseXYEv() });

  m_driveController.SetConstraints(frc::TrapezoidProfile<units::meter>::Constraints {
    units::meters_per_second_t { GetAutoDriveToPoseXYV() },
    units::meters_per_second_squared_t { GetAutoDriveToPoseXYA() },
  });

  m_thetaController.SetP(GetAutoDriveToPoseHP());
  m_thetaController.SetI(GetAutoDriveToPoseHI());
  m_thetaController.SetD(GetAutoDriveToPoseHD());
  m_thetaController.SetTolerance(
    units::degree_t { GetAutoDriveToPoseHEp() },
    units::degrees_per_second_t { GetAutoDriveToPoseHEv() });

  m_thetaController.SetConstraints(frc::TrapezoidProfile<units::radians>::Constraints {
    units::degrees_per_second_t { GetAutoDriveToPoseHV() },
    units::degrees_per_second_squared_t { GetAutoDriveToPoseHA() },
  });

  const auto currentPose = m_drive->GetPose();
  const auto desiredPose = m_desiredPose();

  m_driveController.Reset(currentPose.Translation().Distance(desiredPose.Translation()));
  m_thetaController.Reset(currentPose.Rotation().Radians());
}

void DriveToPoseCommand::Execute() {
  m_running = true;

  const auto currentPose = m_drive->GetPose();
  const auto desiredPose = m_desiredPose();
  const auto currentRotation = currentPose.Rotation();

  auto driveVelocityMagnitude =
    m_driveController.Calculate(currentPose.Translation().Distance(desiredPose.Translation()), 0_m);

  const auto driveAtGoal = m_driveController.AtGoal();
  if (driveAtGoal) {
    driveVelocityMagnitude = 0;
  }

  auto thetaFF = units::radians_per_second_t { m_thetaController.Calculate(
    currentRotation.Radians(), desiredPose.Rotation().Radians()) };

  const auto thetaAtGoal = m_thetaController.AtGoal();
  if (thetaAtGoal) {
    thetaFF = 0_rad_per_s;
  }

  const auto driveVelocity =
    frc::Pose2d(
      frc::Translation2d(), (currentPose.Translation() - desiredPose.Translation()).Angle())
      .TransformBy(frc::Transform2d(
        frc::Translation2d(units::meter_t { driveVelocityMagnitude }, 0_m), frc::Rotation2d()))
      .Translation();

  const auto speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    driveVelocity.X() / 1_s, driveVelocity.Y() / 1_s, thetaFF, currentRotation.Radians());

  m_drive->SetChassisSpeeds(speeds);

#ifdef LOG_AUTO_COMMAND
  // clang-format off
  frc::SmartDashboard::PutNumber("Telemetry/DriveToPose/Distance", m_driveController.GetPositionError().convert<units::meters>().value());
  frc::SmartDashboard::PutNumber("Telemetry/DriveToPose/Theta Error", m_thetaController.GetPositionError().convert<units::degrees>().value());
  frc::SmartDashboard::PutBoolean("Telemetry/DriveToPose/Drive At Goal", driveAtGoal);
  frc::SmartDashboard::PutBoolean("Telemetry/DriveToPose/Theta At Goal", thetaAtGoal);
  // clang-format on
#endif
}

void DriveToPoseCommand::End(bool interrupted) {
  m_running = false;
  m_drive->Stop();
}

bool DriveToPoseCommand::AtGoal() const {
  return m_running && m_driveController.AtGoal() && m_thetaController.AtGoal();
}

bool DriveToPoseCommand::AtGoal(
  units::meter_t distanceTolerance, units::radian_t rotationTolerance) const {
  return m_running && units::math::fabs(m_driveController.GetPositionError()) < distanceTolerance &&
         units::math::fabs(m_thetaController.GetPositionError()) < rotationTolerance;
}

bool DriveToPoseCommand::IsFinished() {
  return m_endAtGoal && AtGoal();
}
