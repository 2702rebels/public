#include "commands/DriveAlignCommand.h"

#include <frc/DataLogManager.h>
#include <frc/geometry/Twist2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <units/velocity.h>

#include "Settings.h"
#include "utils/MathUtils.h"

// clang-format off
REGISTER_DOUBLE(AutoDriveAlignXP,   "Auto/DriveAlign/X - P", 0.8);
REGISTER_DOUBLE(AutoDriveAlignXI,   "Auto/DriveAlign/X - I", 0.0);
REGISTER_DOUBLE(AutoDriveAlignXD,   "Auto/DriveAlign/X - D", 0.0);
REGISTER_DOUBLE(AutoDriveAlignXEp,  "Auto/DriveAlign/X - Ep (m)", 0.05);
REGISTER_DOUBLE(AutoDriveAlignXEv,  "Auto/DriveAlign/X - Ev (mps)", 0.1);
REGISTER_DOUBLE(AutoDriveAlignXV,   "Auto/DriveAlign/X - v", AutoConstants::kMaxSpeed.value());
REGISTER_DOUBLE(AutoDriveAlignXA,   "Auto/DriveAlign/X - a", AutoConstants::kMaxAcceleration.value());
REGISTER_DOUBLE(AutoDriveAlignYP,   "Auto/DriveAlign/Y - P", 2.0);
REGISTER_DOUBLE(AutoDriveAlignYI,   "Auto/DriveAlign/Y - I", 0.0);
REGISTER_DOUBLE(AutoDriveAlignYD,   "Auto/DriveAlign/Y - D", 0.0);
REGISTER_DOUBLE(AutoDriveAlignYEp,  "Auto/DriveAlign/Y - Ep (m)", 0.05);
REGISTER_DOUBLE(AutoDriveAlignYEv,  "Auto/DriveAlign/Y - Ev (mps)", 0.1);
REGISTER_DOUBLE(AutoDriveAlignYV,   "Auto/DriveAlign/Y - v", AutoConstants::kMaxSpeed.value());
REGISTER_DOUBLE(AutoDriveAlignYA,   "Auto/DriveAlign/Y - a", AutoConstants::kMaxAcceleration.value());
REGISTER_DOUBLE(AutoDriveAlignHP,   "Auto/DriveAlign/Theta - P", AutoConstants::kPThetaController);
REGISTER_DOUBLE(AutoDriveAlignHI,   "Auto/DriveAlign/Theta - I", AutoConstants::kIThetaController);
REGISTER_DOUBLE(AutoDriveAlignHD,   "Auto/DriveAlign/Theta - D", AutoConstants::kDThetaController);
REGISTER_DOUBLE(AutoDriveAlignHV,   "Auto/DriveAlign/Theta - v", AutoConstants::kMaxAngularSpeed.value());
REGISTER_DOUBLE(AutoDriveAlignHA,   "Auto/DriveAlign/Theta - a", AutoConstants::kMaxAngularAcceleration.value());
REGISTER_DOUBLE(AutoDriveAlignHEp,  "Auto/DriveAlign/Theta - Ep (deg)", 2);
REGISTER_DOUBLE(AutoDriveAlignHEv,  "Auto/DriveAlign/Theta - Ev (dps)", 10);
REGISTER_DOUBLE(AutoDriveAlignTol,  "Auto/DriveAlign/Tolerance", 1);
REGISTER_DOUBLE(AutoDriveAlignOff,  "Auto/DriveAlign/Offset (m)", 0.05);

// clang-format on

DriveAlignCommand::DriveAlignCommand(
  std::function<std::optional<frc::Pose2d>()> beam,
  std::function<frc::Translation2d()> target,
  DriveSubsystem* drive) :
  m_drive { drive },
  m_beam { beam },
  m_target { target },
  m_alignController {
    frc::ProfiledPIDController<units::meter> {
      GetAutoDriveAlignYP(),
      GetAutoDriveAlignYI(),
      GetAutoDriveAlignYD(),
      frc::TrapezoidProfile<units::meter>::Constraints {
        units::meters_per_second_t { GetAutoDriveAlignYV() },
        units::meters_per_second_squared_t { GetAutoDriveAlignYA() },
      },
    },
  },
  m_driveController {
    frc::ProfiledPIDController<units::meter> {
      GetAutoDriveAlignXP(),
      GetAutoDriveAlignXI(),
      GetAutoDriveAlignXD(),
      frc::TrapezoidProfile<units::meter>::Constraints {
        units::meters_per_second_t { GetAutoDriveAlignXV() },
        units::meters_per_second_squared_t { GetAutoDriveAlignXA() },
      },
    },
  },
  m_thetaController {
    frc::ProfiledPIDController<units::radians> {
      GetAutoDriveAlignHP(),
      GetAutoDriveAlignHI(),
      GetAutoDriveAlignHD(),
      frc::TrapezoidProfile<units::radians>::Constraints {
        units::degrees_per_second_t { GetAutoDriveAlignHV() },
        units::degrees_per_second_squared_t { GetAutoDriveAlignHA() },
      },
    },
  } {
  AddRequirements(drive);
  m_thetaController.EnableContinuousInput(
    units::radian_t { -std::numbers::pi }, units::radian_t { std::numbers::pi });
}

void DriveAlignCommand::Initialize() {
  m_alignController.SetP(GetAutoDriveAlignYP());
  m_alignController.SetI(GetAutoDriveAlignYI());
  m_alignController.SetD(GetAutoDriveAlignYD());
  m_alignController.SetTolerance(
    units::meter_t { GetAutoDriveAlignYEp() },
    units::meters_per_second_t { GetAutoDriveAlignYEv() });

  m_alignController.SetConstraints(frc::TrapezoidProfile<units::meter>::Constraints {
    units::meters_per_second_t { GetAutoDriveAlignYV() },
    units::meters_per_second_squared_t { GetAutoDriveAlignYA() },
  });

  m_driveController.SetP(GetAutoDriveAlignXP());
  m_driveController.SetI(GetAutoDriveAlignXI());
  m_driveController.SetD(GetAutoDriveAlignXD());
  m_driveController.SetTolerance(
    units::meter_t { GetAutoDriveAlignXEp() },
    units::meters_per_second_t { GetAutoDriveAlignXEv() });

  m_driveController.SetConstraints(frc::TrapezoidProfile<units::meter>::Constraints {
    units::meters_per_second_t { GetAutoDriveAlignXV() },
    units::meters_per_second_squared_t { GetAutoDriveAlignXA() },
  });

  m_thetaController.SetP(GetAutoDriveAlignHP());
  m_thetaController.SetI(GetAutoDriveAlignHI());
  m_thetaController.SetD(GetAutoDriveAlignHD());
  m_thetaController.SetTolerance(
    units::degree_t { GetAutoDriveAlignHEp() },
    units::degrees_per_second_t { GetAutoDriveAlignHEv() });

  m_thetaController.SetConstraints(frc::TrapezoidProfile<units::radians>::Constraints {
    units::degrees_per_second_t { GetAutoDriveAlignHV() },
    units::degrees_per_second_squared_t { GetAutoDriveAlignHA() },
  });

  m_tolerance = units::degree_t { GetAutoDriveAlignTol() };
  m_offset = units::meter_t { GetAutoDriveAlignOff() };
  m_aligned = false;
  m_desiredPose = frc::Pose2d();

  const auto beam = m_beam();
  const auto offset = m_target();
  const auto direction = beam.has_value() ? beam.value().Translation() + offset : Zero;

  m_alignController.Reset(-direction.Norm());
  m_thetaController.Reset(m_drive->GetHeading().Radians());
}

void DriveAlignCommand::Execute() {
  m_running = true;

  const auto currentPose = m_drive->GetPose();
  const auto currentRotation = m_drive->GetHeading();

  // we want to position squarely away from the "alliance wall"
  auto thetaFF =
    units::radians_per_second_t { m_thetaController.Calculate(currentRotation.Radians(), 0_deg) };

  const auto thetaAtGoal = m_thetaController.AtGoal();
  if (thetaAtGoal) {
    thetaFF = 0_rad_per_s;
  }

  if (m_aligned) {
    auto driveVelocityMagnitude = m_driveController.Calculate(
      currentPose.Translation().Distance(m_desiredPose.Translation()), 0_m);

    const auto driveAtGoal = m_driveController.AtGoal();
    if (driveAtGoal) {
      driveVelocityMagnitude = 0;
    }

    const auto driveVelocity =
      frc::Pose2d(
        frc::Translation2d(), (currentPose.Translation() - m_desiredPose.Translation()).Angle())
        .TransformBy(frc::Transform2d(
          frc::Translation2d(units::meter_t { driveVelocityMagnitude }, 0_m), frc::Rotation2d()))
        .Translation();

    const auto speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      driveVelocity.X() / 1_s, 0_mps, thetaFF, currentRotation.Radians());

    m_drive->SetChassisSpeeds(speeds);
  } else {
    const auto beam = m_beam();
    const auto offset = m_target();
    const auto direction = beam.has_value() ? beam.value().Translation() + offset : Zero;

    auto driveVelocityMagnitude = m_alignController.Calculate(-direction.Norm(), 0_m);

    // create drive velocity vector based on the direction
    const auto angle = direction.Angle();
    const auto driveVelocity =
      frc::Pose2d(frc::Translation2d(), angle)
        .TransformBy(frc::Transform2d(
          frc::Translation2d(units::meter_t { driveVelocityMagnitude }, 0_m), frc::Rotation2d()))
        .Translation();

    // we have to convert from our local reference to the field
    // positive Y -> positive Vx
    // positive X -> negative Vy

    // keep moving laterally until we are aligned (angle is close to 90°)
    m_aligned = units::math::fabs(90_deg - angle.Degrees()) < m_tolerance;
    m_desiredPose = frc::Pose2d {
      currentPose.Translation() + frc::Translation2d { direction.Y() - m_offset, 0_m },
      currentPose.Rotation(),
    };

    const auto speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      0_mps, -driveVelocity.X() / 1_s, thetaFF, currentRotation.Radians());

    m_drive->SetChassisSpeeds(speeds);
  }

#ifdef LOG_AUTO_COMMAND
  // clang-format off
  frc::SmartDashboard::PutNumber("Telemetry/DriveAlign/Distance", m_driveController.GetPositionError().convert<units::meters>().value());
  frc::SmartDashboard::PutNumber("Telemetry/DriveAlign/Theta Error", m_thetaController.GetPositionError().convert<units::degrees>().value());
  frc::SmartDashboard::PutBoolean("Telemetry/DriveAlign/Theta At Goal", thetaAtGoal);
  frc::SmartDashboard::PutBoolean("Telemetry/DriveAlign/Aligned", m_aligned);
  // clang-format on
#endif
}

void DriveAlignCommand::End(bool interrupted) {
  m_running = false;
  m_drive->Stop();
}

bool DriveAlignCommand::AtGoal() const {
  return m_running && m_driveController.AtGoal() && m_thetaController.AtGoal();
}

bool DriveAlignCommand::AtGoal(
  units::meter_t distanceTolerance, units::radian_t rotationTolerance) const {
  return m_running && units::math::fabs(m_driveController.GetPositionError()) < distanceTolerance &&
         units::math::fabs(m_thetaController.GetPositionError()) < rotationTolerance;
}
