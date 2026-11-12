#include "commands/DriveTrajectoryCommand.h"

#include <frc/DataLogManager.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/math.h>

#include "Settings.h"

// clang-format off
REGISTER_DOUBLE(AutoDriveTrajectoryXP,  "Auto/DriveTrajectory/X - P", AutoConstants::kPXController);
REGISTER_DOUBLE(AutoDriveTrajectoryXI,  "Auto/DriveTrajectory/X - I", AutoConstants::kIXController);
REGISTER_DOUBLE(AutoDriveTrajectoryXD,  "Auto/DriveTrajectory/X - D", AutoConstants::kDXController);
REGISTER_DOUBLE(AutoDriveTrajectoryXE,  "Auto/DriveTrajectory/X - E", 0.05);
REGISTER_DOUBLE(AutoDriveTrajectoryYP,  "Auto/DriveTrajectory/Y - P", AutoConstants::kPYController);
REGISTER_DOUBLE(AutoDriveTrajectoryYI,  "Auto/DriveTrajectory/Y - I", AutoConstants::kIYController);
REGISTER_DOUBLE(AutoDriveTrajectoryYD,  "Auto/DriveTrajectory/Y - D", AutoConstants::kDYController);
REGISTER_DOUBLE(AutoDriveTrajectoryYE,  "Auto/DriveTrajectory/Y - E", 0.05);
REGISTER_DOUBLE(AutoDriveTrajectoryHP,  "Auto/DriveTrajectory/Theta - P", AutoConstants::kPThetaController);
REGISTER_DOUBLE(AutoDriveTrajectoryHI,  "Auto/DriveTrajectory/Theta - I", AutoConstants::kIThetaController);
REGISTER_DOUBLE(AutoDriveTrajectoryHD,  "Auto/DriveTrajectory/Theta - D", AutoConstants::kDThetaController);
REGISTER_DOUBLE(AutoDriveTrajectoryHV,  "Auto/DriveTrajectory/Theta - v", AutoConstants::kMaxAngularSpeed.value());
REGISTER_DOUBLE(AutoDriveTrajectoryHA,  "Auto/DriveTrajectory/Theta - a", AutoConstants::kMaxAngularAcceleration.value());
REGISTER_DOUBLE(AutoDriveTrajectoryHE,  "Auto/DriveTrajectory/Theta - E (deg)", 1.5);
REGISTER_DOUBLE(AutoDriveTrajectoryXYV, "Auto/DriveTrajectory/XY - v", AutoConstants::kMaxSpeed.value());
REGISTER_DOUBLE(AutoDriveTrajectoryXYA, "Auto/DriveTrajectory/XY - a", AutoConstants::kMaxAcceleration.value());

// clang-format on

DriveTrajectoryCommand::DriveTrajectoryCommand(
  const std::vector<Waypoint>& waypoints,
  const frc::Translation2d& origin,
  bool resetOdometry,
  bool stopByTimer,
  bool printTrajectory,
  DriveSubsystem* drive) :
  m_drive { drive },
  m_controller {
    frc2::PIDController {
      GetAutoDriveTrajectoryXP(),
      GetAutoDriveTrajectoryXI(),
      GetAutoDriveTrajectoryXD(),
    },
    frc2::PIDController {
      GetAutoDriveTrajectoryYP(),
      GetAutoDriveTrajectoryYI(),
      GetAutoDriveTrajectoryYD(),
    },
    frc::PIDController {
      GetAutoDriveTrajectoryHP(),
      GetAutoDriveTrajectoryHI(),
      GetAutoDriveTrajectoryHD(),
    },
  },
  m_resetOdometry { resetOdometry },
  m_stopByTimer { stopByTimer } {
  AddRequirements(drive);

  // TODO: error handler
  m_trajectory = WaypointTrajectory(
    waypoints,
    origin,
    drive->kDriveKinematics,
    units::meters_per_second_t { GetAutoDriveTrajectoryXYV() },
    units::meters_per_second_squared_t { GetAutoDriveTrajectoryXYA() },
    units::degrees_per_second_t { GetAutoDriveTrajectoryHV() },
    units::degrees_per_second_squared_t { GetAutoDriveTrajectoryHA() });

  if (printTrajectory) {
    frc::DataLogManager::Log(fmt::format("DriveTrajectory: Trajectory"));
    for (const auto& state : m_trajectory.Trajectory().States()) {
      frc::DataLogManager::Log(fmt::format(
        "at {:.3f} -> ({:.2f}, {:.2f}, {:.1f}) v = {:.2f} a = {:.2f}",
        state.t.value(),
        state.pose.X().value(),
        state.pose.Y().value(),
        state.pose.Rotation().Degrees().value(),
        state.velocity.value(),
        state.acceleration.value()));
    }

    const auto rotationStates = m_trajectory.Rotations().States();
    if (rotationStates.size() > 0) {
      frc::DataLogManager::Log(fmt::format("DriveTrajectory: Rotation Profile"));
      for (const auto& state : rotationStates) {
        frc::DataLogManager::Log(fmt::format(
          "at {:.3f} -> {:.1f} v = {:.2f}",
          state.time.value(),
          state.rotation.Degrees().value(),
          state.velocity.value()));
      }
    }
  }
}

void DriveTrajectoryCommand::Initialize() {
  // reset odometry to the starting pose of the trajectory
  if (m_resetOdometry) {
    m_drive->ResetOdometry(m_trajectory.InitialPose());
  }

  m_controller.getXController().SetP(GetAutoDriveTrajectoryXP());
  m_controller.getXController().SetI(GetAutoDriveTrajectoryXI());
  m_controller.getXController().SetD(GetAutoDriveTrajectoryXD());

  m_controller.getYController().SetP(GetAutoDriveTrajectoryYP());
  m_controller.getYController().SetI(GetAutoDriveTrajectoryYI());
  m_controller.getYController().SetD(GetAutoDriveTrajectoryYD());

  m_controller.getThetaController().SetP(GetAutoDriveTrajectoryHP());
  m_controller.getThetaController().SetI(GetAutoDriveTrajectoryHI());
  m_controller.getThetaController().SetD(GetAutoDriveTrajectoryHD());

  m_controller.SetTolerance(frc::Pose2d {
    units::meter_t { GetAutoDriveTrajectoryXE() },
    units::meter_t { GetAutoDriveTrajectoryYE() },
    units::degree_t { GetAutoDriveTrajectoryHE() },
  });

  m_controller.Reset();

  m_finalPose = m_trajectory.FinalPose();
  m_timer.Restart();
}

void DriveTrajectoryCommand::Execute() {
  m_running = true;
  const auto time = m_timer.Get();

  const auto currentPose = m_drive->GetPose();
  const auto desiredState = m_trajectory.Trajectory().Sample(time);
  const auto desiredRotation = m_trajectory.Rotations().Sample(time);

  const auto speeds = m_controller.Calculate(currentPose, desiredState, desiredRotation);
  m_drive->SetChassisSpeeds(speeds);

#ifdef LOG_AUTO_COMMAND
  const auto distanceError = desiredState.pose.Translation().Distance(currentPose.Translation());
  const auto thetaError =
    MathUtils::ClampAngle(desiredRotation.rotation.Radians() - currentPose.Rotation().Radians());

  // clang-format off
  frc::SmartDashboard::PutNumber("Telemetry/DriveTrajectory/Desired X", desiredState.pose.X().convert<units::meters>().value());
  frc::SmartDashboard::PutNumber("Telemetry/DriveTrajectory/Desired Y", desiredState.pose.Y().convert<units::meters>().value());
  frc::SmartDashboard::PutNumber("Telemetry/DriveTrajectory/Desired V", desiredState.velocity.convert<units::meters_per_second>().value());
  frc::SmartDashboard::PutNumber("Telemetry/DriveTrajectory/Desired Θ", desiredRotation.rotation.Degrees().convert<units::degrees>().value());
  frc::SmartDashboard::PutNumber("Telemetry/DriveTrajectory/Desired ω", desiredRotation.velocity.convert<units::degrees_per_second>().value());
  frc::SmartDashboard::PutNumber("Telemetry/DriveTrajectory/Distance Error", distanceError.convert<units::meters>().value());
  frc::SmartDashboard::PutNumber("Telemetry/DriveTrajectory/Theta Error", thetaError.convert<units::degrees>().value());
  frc::SmartDashboard::PutBoolean("Telemetry/DriveTrajectory/At Goal", AtGoal());
  // clang-format on
#endif
}

void DriveTrajectoryCommand::End(bool interrupted) {
  m_running = false;
  m_timer.Stop();

  if (m_trajectory.GetEndVelocity() == 0_mps) {
    m_drive->Stop();
  }
}

bool DriveTrajectoryCommand::AtGoal() const {
  if (!m_running) {
    return false;
  }

  const auto currentPose = m_drive->GetPose();
  const auto tolerance = m_controller.GetTolerance();

  const auto eTranslateX = m_finalPose.X() - currentPose.X();
  const auto eTranslateY = m_finalPose.Y() - currentPose.Y();
  const auto eRotate = m_finalPose.Rotation() - currentPose.Rotation();
  const auto tolTranslate = tolerance.Translation();
  const auto tolRotate = tolerance.Rotation();

  return units::math::fabs(eTranslateX) < tolTranslate.X() &&
         units::math::fabs(eTranslateY) < tolTranslate.Y() &&
         units::math::fabs(eRotate.Radians()) < tolRotate.Radians();
}

bool DriveTrajectoryCommand::IsFinished() {
  return m_stopByTimer ? m_timer.HasElapsed(m_trajectory.Trajectory().TotalTime()) : AtGoal();
}
