// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Constants.h>
#include <frc/DigitalInput.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "subsystems/IntakeSubsystem.h"

enum ArmPosition {
  STOWED,
  INTAKE_FLOOR_CONE,
  INTAKE_FLOOR_CUBE,
  INTAKE_STATION_CONE,
  INTAKE_STATION_CUBE,
  SCORE_LOW_CONE,
  SCORE_LOW_CUBE,
  SCORE_MID_CONE,
  SCORE_MID_CUBE,
  SCORE_HIGH_CONE,
  SCORE_HIGH_CUBE,
};

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();
  void ConfigureTuningParams();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void OnDisable();

  units::degree_t GetAxis1Angle();
  units::degrees_per_second_t GetAxis1Velocity();
  units::inch_t GetAxis2Distance();
  units::meters_per_second_t GetAxis2Velocity();

  bool GetAxis1LimitSwitch();

  double GetAxis1Current();
  double GetAxis2Current();

  void SetAxis1Off();
  void SetAxis2Off();
  void SetArmOff();

  void ZeroAxis1();
  void ZeroAxis2();

  void SetAxis1DutyCycle(double dutyCycle);
  void SetAxis2DutyCycle(double dutyCycle);

  void Move(units::degree_t angle);
  void Move(units::inch_t distance);
  void Move(units::degree_t angle, units::inch_t distance);
  void Move(ArmPosition position);
  bool IsAtCommandedPosition();

  frc2::CommandPtr JogUp();
  frc2::CommandPtr JogDown();

  void ReloadConfigs();

  void DisableControlAxis2();

  bool IsCompBot() {
    return m_whichRobotJumper.Get();
  }

  bool IsArmUp();

  units::meters_per_second_t GetMaxSpeedOverride() const;

 private:
  bool m_isAxis1ControlEnabled = false;
  bool m_isAxis2ControlEnabled = false;

  units::degree_t m_armUpThreshold;
  units::degree_t m_axis1SetPoint;
  units::inch_t m_axis2SetPoint;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_axis1Motor { ArmConstants::kExtendingArmAxis1MotorControllerCanId,
                                  rev::CANSparkMax::MotorType::kBrushless };
  rev::CANSparkMax m_axis2Motor { ArmConstants::kExtendingArmAxis2MotorControllerCanId,
                                  rev::CANSparkMax::MotorType::kBrushless };

  rev::SparkMaxPIDController m_axis1PIDController = m_axis1Motor.GetPIDController();
  rev::SparkMaxPIDController m_axis2PIDController = m_axis2Motor.GetPIDController();

  rev::SparkMaxRelativeEncoder m_axis1Encoder = m_axis1Motor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_axis2Encoder = m_axis2Motor.GetEncoder();

  frc::DigitalInput m_axis1HomeLimitSwitch { ArmConstants::kAxis1HomeLimitSwitchId };
  frc::DigitalInput m_whichRobotJumper { ArmConstants::kWhichRobotJumper };

  frc::ArmFeedforward* m_axis1feedforward;
  // frc::TrapezoidProfile<units::degree>* m_axis1TrapezoidalProfile;
  frc::TrapezoidProfile<units::degree>::Constraints* m_axis1TrapezoidalProfileConstraints;
  frc::TrapezoidProfile<units::degree>::State m_axis1TrapezoidalProfileState;

  // frc::TrapezoidProfile<units::meters>* m_axis2TrapezoidalProfile;
  frc::TrapezoidProfile<units::meters>::Constraints* m_axis2TrapezoidalProfileConstraints;
  frc::TrapezoidProfile<units::meters>::State m_axis2TrapezoidalProfileState;

  void Axis1Control();
  void Axis2Control();
};
