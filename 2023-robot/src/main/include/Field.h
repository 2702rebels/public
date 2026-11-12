#pragma once

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>

#include <vector>

#include "Constants.h"

class Field : public frc2::SubsystemBase {
 public:
  enum Stations {
    CONE_1,
    CUBE_1,
    CONE_2,
    CONE_3,
    CUBE_2,
    CONE_4,
    CONE_5,
    CUBE_3,
    CONE_6,
    LOAD_L,
    LOAD_R
  };

  Field();
  void Periodic() override;

  /**
   * Shifts selection of the target station.
   */
  void ShiftSelection(double inputX, double inputY);

  /**
   * Selects the target score station.
   */
  void SelectScoreStation(Stations station);

  /**
   * Selects the target load station.
   */
  void SelectLoadStation(Stations station);

  /**
   * Returns the position based on the currently selected station and alliance from DS.
   */
  frc::Pose2d GetScorePosition() const;

  /**
   * Returns the position based on the currently selected station and alliance from DS.
   */
  frc::Pose2d GetLoadPosition() const;

  /**
   * Returns the offset from the loading station vision tag to the target position
   * based on the currently selected station abd alliance from DS.
   */
  frc::Translation2d GetLoadOffset() const;

  /**
   * Returns the position on the field for the specified game piece.
   *
   * The position is relative to the "alliance wall", with Y = 0 being
   * the field corner to right of the driver.
   */
  static frc::Translation2d GetGamePiece(int index);

  /**
   * Returns the position on the field for the specified station.
   *
   * The position is relative to the "alliance wall", with Y = 0 being
   * the field corner to right of the driver.
   *
   * The returned pose assumes the robot is facing the wall, which
   * is either the "alliance wall" (180°) or "loading" (0°).
   */
  static frc::Pose2d GetFieldPose(Stations station);

  /**
   * Translates global field pose to the pose relative to the "alliance wall".
   */
  static frc::Pose2d ToAlliancePose(const frc::Pose2d& pose);

 private:
  inline constexpr static units::meter_t FieldLength = 651.25_in;
  inline constexpr static units::meter_t FieldWidth = 315.5_in;
  inline constexpr static units::meter_t StationShift = 20.19_in;
  inline constexpr static units::meter_t StationDepth = 54.45_in;
  inline constexpr static units::meter_t StationDelta = 22_in;
  inline constexpr static units::meter_t LoadingDepth = 21_in;  // accounts for arm extension
  inline constexpr static units::meter_t LoadingOffY = 31.45_in;
  inline constexpr static units::meter_t LoadingOffX = 6_in;
  inline constexpr static units::meter_t LoadingTagY = 265.74_in;

  // these position coordinates assume BLUE alliance wall being the origin
  constexpr static frc::Translation2d Positions[] = {
    { StationDepth, StationShift + StationDelta * 0 },          // CONE_1
    { StationDepth, StationShift + StationDelta * 1 },          // CUBE_1
    { StationDepth, StationDelta + StationDelta * 2 },          // CONE_2
    { StationDepth, StationDelta + StationDelta * 3 },          // CONE_3
    { StationDepth, StationDelta + StationDelta * 4 },          // CUBE_2
    { StationDepth, StationDelta + StationDelta * 5 },          // CONE_4
    { StationDepth, StationDelta + StationDelta * 6 },          // CONE_5
    { StationDepth, StationDelta + StationDelta * 7 },          // CUBE_3
    { StationDepth, StationDelta + StationDelta * 8 },          // CONE_6
    { FieldLength - LoadingDepth, LoadingTagY + LoadingOffY },  // LOAD_L
    { FieldLength - LoadingDepth, LoadingTagY - LoadingOffY },  // LOAD_R
  };

  inline constexpr static units::meter_t GamePieceX = FieldLength / 2 - 47.36_in;
  inline constexpr static units::meter_t GamePieceY = 36.19_in;
  inline constexpr static units::meter_t GamePieceYDelta = 48_in;

  // position of game pieces
  constexpr static frc::Translation2d GamePieces[] = {
    { GamePieceX, GamePieceY + GamePieceYDelta * 0 },
    { GamePieceX, GamePieceY + GamePieceYDelta * 1 },
    { GamePieceX, GamePieceY + GamePieceYDelta * 2 },
    { GamePieceX, GamePieceY + GamePieceYDelta * 3 },
  };

  Stations m_stationScore = Stations::CUBE_3;
  Stations m_stationLoad = Stations::LOAD_L;
  units::second_t m_lastShiftedAt;

  // used for telemetry
  double m_targetScore[3];
  double m_targetLoad[3];
};
