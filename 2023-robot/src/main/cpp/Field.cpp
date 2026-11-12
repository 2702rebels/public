#include "Field.h"

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "utils/MathUtils.h"

Field::Field() {
  SelectScoreStation(Stations::CUBE_3);
  SelectLoadStation(Stations::LOAD_L);
}

void Field::Periodic() {
  frc::SmartDashboard::PutBoolean("Cone 1", m_stationScore == Stations::CONE_1);
  frc::SmartDashboard::PutBoolean("Cone 2", m_stationScore == Stations::CONE_2);
  frc::SmartDashboard::PutBoolean("Cone 3", m_stationScore == Stations::CONE_3);
  frc::SmartDashboard::PutBoolean("Cone 4", m_stationScore == Stations::CONE_4);
  frc::SmartDashboard::PutBoolean("Cone 5", m_stationScore == Stations::CONE_5);
  frc::SmartDashboard::PutBoolean("Cone 6", m_stationScore == Stations::CONE_6);
  frc::SmartDashboard::PutBoolean("Cube 1", m_stationScore == Stations::CUBE_1);
  frc::SmartDashboard::PutBoolean("Cube 2", m_stationScore == Stations::CUBE_2);
  frc::SmartDashboard::PutBoolean("Cube 3", m_stationScore == Stations::CUBE_3);
  frc::SmartDashboard::PutBoolean("Load L", m_stationLoad == Stations::LOAD_L);
  frc::SmartDashboard::PutBoolean("Load R", m_stationLoad == Stations::LOAD_R);
  frc::SmartDashboard::PutNumberArray("Target Score", m_targetScore);
  frc::SmartDashboard::PutNumberArray("Target Load", m_targetLoad);
}

frc::Translation2d Field::GetGamePiece(int index) {
  const auto position = GamePieces[index];
  return frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue
         ? position
         : frc::Translation2d(position.X(), FieldWidth - position.Y());
}

frc::Pose2d Field::GetFieldPose(Stations station) {
  const auto alliance = frc::DriverStation::GetAlliance();
  const auto isLoadingStation = station == Stations::LOAD_L || station == Stations::LOAD_R;
  const auto orientation = isLoadingStation ? 0_deg : 180_deg;

  // positions coordinates are for the front of the robot, we adjust
  // to translate to the robot's center and also add some safety distance
  const auto adjustment =
    (isLoadingStation ? -1.0 : 1.0) * ((RobotConstants::kBumperSize / 2) + 5_cm);

  if (alliance == frc::DriverStation::Alliance::kBlue) {
    const auto position = Positions[station];
    return frc::Pose2d(frc::Translation2d(position.X() + adjustment, position.Y()), orientation);
  } else {
    // for RED alliance swap right and left loading stations
    if (isLoadingStation) {
      station = station == Stations::LOAD_R ? Stations::LOAD_L : Stations::LOAD_R;
    }

    const auto position = Positions[station];
    return frc::Pose2d(
      frc::Translation2d((position.X() + adjustment), FieldWidth - position.Y()), orientation);
  }
}

frc::Translation2d Field::GetLoadOffset() const {
  const auto alliance = frc::DriverStation::GetAlliance();

  auto station = m_stationLoad;

  // for RED alliance swap right and left stations
  if (alliance == frc::DriverStation::Alliance::kRed) {
    station = station == Stations::LOAD_R ? Stations::LOAD_L : Stations::LOAD_R;
  }

  return frc::Translation2d {
    station == Stations::LOAD_L ? LoadingOffY : -LoadingOffY,
    -LoadingOffX,
  };
}

frc::Pose2d Field::ToAlliancePose(const frc::Pose2d& pose) {
  const auto alliance = frc::DriverStation::GetAlliance();
  if (alliance == frc::DriverStation::Alliance::kBlue) {
    return pose;
  }

  const units::radian_t orientation = pose.Rotation().Degrees() + 180_deg;
  return frc::Pose2d(
    frc::Translation2d(FieldLength - pose.X(), FieldWidth - pose.Y()),
    frc::Rotation2d(MathUtils::ClampAngle(orientation)));
}

frc::Pose2d Field::GetScorePosition() const {
  return Field::GetFieldPose(m_stationScore);
}

frc::Pose2d Field::GetLoadPosition() const {
  return Field::GetFieldPose(m_stationLoad);
}

void Field::SelectScoreStation(Stations station) {
  m_stationScore = station;

  // update telemetry
  const auto target = GetScorePosition();
  m_targetScore[0] = target.X().value();
  m_targetScore[1] = target.Y().value();
  m_targetScore[2] = target.Rotation().Degrees().value();
}

void Field::SelectLoadStation(Stations station) {
  m_stationLoad = station;

  // update telemetry
  const auto target = GetLoadPosition();
  m_targetLoad[0] = target.X().value();
  m_targetLoad[1] = target.Y().value();
  m_targetLoad[2] = target.Rotation().Degrees().value();
}

void Field::ShiftSelection(double inputX, double inputY) {
  if (std::fabs(inputX) >= 0.5) {
    const auto timestamp = frc::Timer::GetFPGATimestamp();
    if (timestamp - m_lastShiftedAt < 0.2_s) {
      return;
    }

    m_lastShiftedAt = timestamp;

    if (inputX < 0) {
      if (m_stationScore > 0) {
        SelectScoreStation((Stations) (m_stationScore - 1));
      }
    } else {
      if (m_stationScore < Stations::CONE_6) {
        SelectScoreStation((Stations) (m_stationScore + 1));
      }
    }
  }

  if (std::fabs(inputY) >= 0.5) {
    SelectLoadStation(inputY > 0 ? Stations::LOAD_L : Stations::LOAD_R);
  }
}
