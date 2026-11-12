// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

using namespace IntakeConstants;

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void ConfigureTuningParams();
  void OnDisable();

  void SetIntakeSpeed(double speed);
  void IntakeCone();
  void IntakeCube();
  void OuttakeCone();
  void OuttakeCube();
  void PlaceCone();
  void Stop();
  void HoldGamePiece();

  void OpenCone();
  void CloseCone();
  void OpenCube();
  void OpenAuto();
  void CloseCube();
  bool IsAtCommandedPosition();
  bool IsGamePieceSwitchPressed();

  units::degree_t GetGripperAngle();
  double GetIntakeCurrent();
  bool GetCurrentAboveValue(double val);
  double GetGripperCurrent();
  void SetGripperOff();
  void ZeroGripper();
  void SetGripperNextSetpoint(units::degree_t setPoint);
  void SetGripperDutyCycle(double dutyCycle);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  bool m_isHasGamePiece = false;
  bool m_isGripperControlEnabled = false;

  TalonSRX m_leaderMotorController { kIntakeLeaderControllerCanId };
  VictorSPX m_followerMotorController { kIntakeFollowerControllerCanId };

  TalonSRX m_gripperMotor { ArmConstants::kExtendingArmAxis3MotorControllerCanId };
  frc2::PIDController m_gripperMotorPIDController { 0, 0, 0 };
  frc::Encoder m_gripperMotorEncoder { IntakeConstants::kGripperEncoderChannel1Id,
                                       IntakeConstants::kGripperEncoderChannel2Id };

  frc::DigitalInput m_limitSwitch { IntakeConstants::kIntakeSwitchId };

  double m_gripperMotorFeedForwards = 0;
};
