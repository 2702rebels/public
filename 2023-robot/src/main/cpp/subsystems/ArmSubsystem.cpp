// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include "Settings.h"

using namespace ArmConstants;
using namespace frc2;

REGISTER_DOUBLE(Axis1P, "ExArm/TuningParameters/Axis1 PID/P", 0.05)
REGISTER_DOUBLE(Axis1I, "ExArm/TuningParameters/Axis1 PID/I", 0.0)
REGISTER_DOUBLE(Axis1D, "ExArm/TuningParameters/Axis1 PID/D", 0.0)
REGISTER_DOUBLE(Axis1kS, "ExArm/TuningParameters/Axis1 ArmFeedForward/kS", 0.0)
REGISTER_DOUBLE(Axis1kG, "ExArm/TuningParameters/Axis1 ArmFeedForward/kG", 0.0)
REGISTER_DOUBLE(Axis1kV, "ExArm/TuningParameters/Axis1 ArmFeedForward/kV", 0.0)
REGISTER_DOUBLE(Axis1kA, "ExArm/TuningParameters/Axis1 ArmFeedForward/kA", 0.0)
REGISTER_DOUBLE(
  Axis1MaxVelocity, "ExArm/TuningParameters/Axis1 MotionProfile/MaxVelDegPerSecond", 100.0)
REGISTER_DOUBLE(
  Axis1MaxAcceletration, "ExArm/TuningParameters/Axis1 MotionProfile/MaxAccelDegPerSecond", 130.0)
REGISTER_DOUBLE(Axis1CurrentLimit, "ExArm/TuningParameters/Axis1CurrentLimit", 40.0)

REGISTER_DOUBLE(Axis2P, "ExArm/TuningParameters/Axis2 PID/P", 0.04)
REGISTER_DOUBLE(Axis2I, "ExArm/TuningParameters/Axis2 PID/I", 0.00001)
REGISTER_DOUBLE(Axis2D, "ExArm/TuningParameters/Axis2 PID/D", 0.0)
REGISTER_DOUBLE(
  Axis2MaxVelocity, "ExArm/TuningParameters/Axis2 MotionProfile/MaxVelInchPerSec", 40.0)
REGISTER_DOUBLE(
  Axis2MaxAcceletration, "ExArm/TuningParameters/Axis2 MotionProfile/MaxAccelInchPerSec", 60.0)
REGISTER_DOUBLE(Axis2CurrentLimit, "ExArm/TuningParameters/Axis2CurrentLimit", 20.0)

REGISTER_DOUBLE(ConeAxis1IntakeFloor, "ExArm/Position/Cone/IntakeFloor/Axis1", 18.0)
REGISTER_DOUBLE(ConeAxis2IntakeFloor, "ExArm/Position/Cone/IntakeFloor/Axis2", 14.0)
REGISTER_DOUBLE(ConeAxis1IntakeStation, "ExArm/Position/Cone/IntakeStation/Axis1", 59.0)
REGISTER_DOUBLE(ConeAxis2IntakeStation, "ExArm/Position/Cone/IntakeStation/Axis2", 0)

REGISTER_DOUBLE(ConeAxis1ScoreLow, "ExArm/Position/Cone/ScoreLow/Axis1", 18.0)
REGISTER_DOUBLE(ConeAxis2ScoreLow, "ExArm/Position/Cone/ScoreLow/Axis2", 0.0)
REGISTER_DOUBLE(ConeAxis1ScoreMid, "ExArm/Position/Cone/ScoreMid/Axis1", 62.0)
REGISTER_DOUBLE(ConeAxis2ScoreMid, "ExArm/Position/Cone/ScoreMid/Axis2", 7.0)
REGISTER_DOUBLE(ConeAxis1ScoreHigh, "ExArm/Position/Cone/ScoreHigh/Axis1", 78.0)
REGISTER_DOUBLE(ConeAxis2ScoreHigh, "ExArm/Position/Cone/ScoreHigh/Axis2", 20.0)

REGISTER_DOUBLE(CubeAxis1IntakeFloor, "ExArm/Position/Cube/IntakeFloor/Axis1", 18.0)
REGISTER_DOUBLE(CubeAxis2IntakeFloor, "ExArm/Position/Cube/IntakeFloor/Axis2", 14.0)
REGISTER_DOUBLE(CubeAxis1IntakeStation, "ExArm/Position/Cube/IntakeStation/Axis1", 56.0)
REGISTER_DOUBLE(CubeAxis2IntakeStation, "ExArm/Position/Cube/IntakeStation/Axis2", 5.0)

REGISTER_DOUBLE(CubeAxis1ScoreLow, "ExArm/Position/Cube/ScoreLow/Axis1", 18.0)
REGISTER_DOUBLE(CubeAxis2ScoreLow, "ExArm/Position/Cube/ScoreLow/Axis2", 0.0)
REGISTER_DOUBLE(CubeAxis1ScoreMid, "ExArm/Position/Cube/ScoreMid/Axis1", 52.0)
REGISTER_DOUBLE(CubeAxis2ScoreMid, "ExArm/Position/Cube/ScoreMid/Axis2", 0.0)
REGISTER_DOUBLE(CubeAxis1ScoreHigh, "ExArm/Position/Cube/ScoreHigh/Axis1", 70.0)
REGISTER_DOUBLE(CubeAxis2ScoreHigh, "ExArm/Position/Cube/ScoreHigh/Axis2", 10.0)

REGISTER_DOUBLE(JogDistance, "ExArm/JogDistance", 5.0);
REGISTER_DOUBLE(DriveMaxSpeedOverride, "ExArm/Max Speed Override (mps)", 2.0);
REGISTER_DOUBLE(ArmUpThreshold, "ExArm/Arm Up Threshold", 15.0);

ArmSubsystem::ArmSubsystem() {
  frc::DataLogManager::Log(fmt::format("Initializing ArmSubsystem."));
  ArmSubsystem::ConfigureTuningParams();
}

void ArmSubsystem::ConfigureTuningParams() {
  // Axis 1.
  m_axis1Motor.RestoreFactoryDefaults();
  m_axis1Motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_axis1Motor.SetInverted(false);
  m_axis1Motor.SetSmartCurrentLimit(GetAxis1CurrentLimit());
  m_axis1PIDController.SetP(GetAxis1P());
  m_axis1PIDController.SetI(GetAxis1I());
  m_axis1PIDController.SetD(GetAxis1D());
  m_axis1PIDController.SetOutputRange(kAxis1MinOutput, kAxis1MaxOutput);
  // Conver to degree's per second.
  if (ArmSubsystem::IsCompBot()) {
    m_axis1Encoder.SetVelocityConversionFactor((1 / kAxis1MotorRotationsPerDegreeComp) / 60);
  } else {
    m_axis1Encoder.SetVelocityConversionFactor((1 / kAxis1MotorRotationsPerDegree) / 60);
  }
  m_axis1Motor.BurnFlash();
  m_axis1feedforward = new frc::ArmFeedforward(
    units::volt_t(GetAxis1kS()),
    units::volt_t(GetAxis1kG()),
    units::unit_t<frc::ArmFeedforward::kv_unit>(GetAxis1kV()),
    units::unit_t<frc::ArmFeedforward::ka_unit>(GetAxis1kA()));
  m_axis1TrapezoidalProfileConstraints = new frc::TrapezoidProfile<units::degree>::Constraints(
    units::degrees_per_second_t(GetAxis1MaxVelocity()),
    units::degrees_per_second_squared_t(GetAxis1MaxAcceletration()));
  frc::DataLogManager::Log(fmt::format(
    "Configured Axis1 with parameters: P={}, I={}, D={}, CurrentLimit={}, MinOutput={}, "
    "MaxOutput={}, ArmFeedForward=[kS={}, kG={}, kV={}, kA={}], MaxVelocity={}, MaxAcceleration={}",
    GetAxis1P(),
    GetAxis1I(),
    GetAxis1D(),
    GetAxis1CurrentLimit(),
    kAxis1MinOutput,
    kAxis1MaxOutput,
    GetAxis1kS(),
    GetAxis1kG(),
    GetAxis1kV(),
    GetAxis1kA(),
    GetAxis1MaxVelocity(),
    GetAxis1MaxAcceletration()));

  // Axis 2
  m_axis2Motor.RestoreFactoryDefaults();
  m_axis2Motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_axis2Motor.SetInverted(false);
  m_axis2Motor.SetSmartCurrentLimit(GetAxis2CurrentLimit());
  m_axis2PIDController.SetP(GetAxis2P());
  m_axis2PIDController.SetI(GetAxis2I());
  m_axis2PIDController.SetD(GetAxis2D());
  m_axis2PIDController.SetOutputRange(kAxis2MinOutput, kAxis2MaxOutput);
  // Convert to meters per second.
  m_axis2Encoder.SetVelocityConversionFactor((1 / kAxis2MotorRotationsPerMeterOfTravel) / 60);
  m_axis2Motor.BurnFlash();
  m_axis2TrapezoidalProfileConstraints = new frc::TrapezoidProfile<units::meter>::Constraints(
    units::meters_per_second_t(GetAxis2MaxVelocity() * 0.0254),
    units::meters_per_second_squared_t(GetAxis2MaxAcceletration() * 0.0254));
  frc::DataLogManager::Log(fmt::format(
    "Configured Axis2 with parameters: P={}, I={}, D={}, CurrentLimit={}, MinOutput={}, "
    "MaxOutput={},  MaxVelocity={}, MaxAcceleration={}",
    GetAxis2P(),
    GetAxis2I(),
    GetAxis2D(),
    GetAxis2CurrentLimit(),
    kAxis2MinOutput,
    kAxis2MaxOutput,
    GetAxis2MaxVelocity(),
    GetAxis2MaxAcceletration()));

  m_armUpThreshold = units::degree_t { GetArmUpThreshold() };

  m_axis1PIDController.SetP(0, 1);
  m_axis1PIDController.SetI(0, 1);
  m_axis1PIDController.SetD(0, 1);
}

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("Axis1Angle", GetAxis1Angle().value());
  frc::SmartDashboard::PutNumber("Axis2Distance", GetAxis2Distance().value());
  frc::SmartDashboard::PutNumber("Axis1Current", GetAxis1Current());
  frc::SmartDashboard::PutNumber("Axis2Current", GetAxis2Current());
  frc::SmartDashboard::PutBoolean("Axis1HomeLimitSwitch", GetAxis1LimitSwitch());
  frc::SmartDashboard::PutNumber("Axis1SetPoint", m_axis1SetPoint.value());
  frc::SmartDashboard::PutNumber("Axis2SetPoint", m_axis2SetPoint.value());
  frc::SmartDashboard::PutBoolean("IsComp", IsCompBot());
  frc::SmartDashboard::PutBoolean("IsArmAtPosition", IsAtCommandedPosition());

  ArmSubsystem::Axis1Control();
  ArmSubsystem::Axis2Control();
}

void ArmSubsystem::Axis1Control() {
  units::degree_t axis1ProfileSetPoint = m_axis1SetPoint;
  if (
    axis1ProfileSetPoint.value() < kPastBumperAngle &&
    GetAxis2Distance().value() > kAxis2ZeroDeadBand) {
    axis1ProfileSetPoint = units::degree_t(kPastBumperAngle);
  }

  // Axis 1 control loop.
  if (m_isAxis1ControlEnabled) {
    frc::TrapezoidProfile<units::degree> axis1Profile {
      *m_axis1TrapezoidalProfileConstraints,
      frc::TrapezoidProfile<units::degree>::State { axis1ProfileSetPoint,
                                                    units::degrees_per_second_t(0) },
      m_axis1TrapezoidalProfileState
    };

    m_axis1TrapezoidalProfileState = axis1Profile.Calculate(ArmConstants::kDt);
    double rotations =
      m_axis1TrapezoidalProfileState.position.value() * kAxis1MotorRotationsPerDegree;
    if (ArmSubsystem::IsCompBot()) {
      rotations =
        m_axis1TrapezoidalProfileState.position.value() * kAxis1MotorRotationsPerDegreeComp;
    }
#ifdef VERBOSE_LOG
    frc::DataLogManager::Log(
      fmt::format("Axis1: Setting setpoint to: {}, Setting rotations to: {}", setPoint, rotations));
#endif

#ifdef ARMFEEDFORWARDS_ENABLED
    m_axis1PIDController.SetReference(
      rotations,
      rev::CANSparkMax::ControlType::kPosition,
      0,
      m_axis1feedforward
        ->Calculate(
          ArmSubsystem::GetAxis1Angle() - units::degree_t(kaxis1ffAngleCorrection),
          units::radians_per_second_t(0),
          units::radians_per_second_squared_t(0))
        .value());
#else
    m_axis1PIDController.SetReference(rotations, rev::CANSparkMax::ControlType::kPosition);
#endif
  }
}

void ArmSubsystem::Axis2Control() {
  units::inch_t axis2ProfileSetPoint = m_axis2SetPoint;
  if (GetAxis1Angle().value() < kPastBumperAngle) {
    axis2ProfileSetPoint = units::inch_t(0);
  }

  // Axis 2 control loop.
  if (m_isAxis2ControlEnabled) {
    frc::TrapezoidProfile<units::meters> axis2Profile {
      *m_axis2TrapezoidalProfileConstraints,
      frc::TrapezoidProfile<units::meters>::State { axis2ProfileSetPoint, 0_mps },
      m_axis2TrapezoidalProfileState
    };

    m_axis2TrapezoidalProfileState = axis2Profile.Calculate(ArmConstants::kDt);
    double rotations = m_axis2TrapezoidalProfileState.position.convert<units::inch>().value() *
                       kAxis2MotorRotationsPerInchOfTravel;
#ifdef VERBOSE_LOG
    frc::DataLogManager::Log(
      fmt::format("Axis2: Setting setpoint to: {}, Setting rotations to: {}", setPoint, rotations));
#endif
    m_axis2PIDController.SetReference(rotations, rev::CANSparkMax::ControlType::kPosition);
    // frc::DataLogManager::Log(fmt::format(
    //   "SetReference Rotations={}, Location={}",
    //   rotations,
    //   ArmSubsystem::GetAxis2Distance().value()));
  }
}

void ArmSubsystem::OnDisable() {
  m_isAxis1ControlEnabled = false;
  m_isAxis2ControlEnabled = false;
  ArmSubsystem::SetArmOff();
}

units::degree_t ArmSubsystem::GetAxis1Angle() {
  if (ArmSubsystem::IsCompBot()) {
    return units::degree_t(m_axis1Encoder.GetPosition() / kAxis1MotorRotationsPerDegreeComp);
  } else {
    return units::degree_t(m_axis1Encoder.GetPosition() / kAxis1MotorRotationsPerDegree);
  }
}

units::degrees_per_second_t ArmSubsystem::GetAxis1Velocity() {
  return units::degrees_per_second_t(m_axis1Encoder.GetVelocity());
}

units::inch_t ArmSubsystem::GetAxis2Distance() {
  return units::inch_t(m_axis2Encoder.GetPosition() / kAxis2MotorRotationsPerInchOfTravel);
}

units::meters_per_second_t ArmSubsystem::GetAxis2Velocity() {
  return units::meters_per_second_t(m_axis2Encoder.GetVelocity());
}

bool ArmSubsystem::GetAxis1LimitSwitch() {
  return m_axis1HomeLimitSwitch.Get();
}

double ArmSubsystem::GetAxis1Current() {
  return m_axis1Motor.GetOutputCurrent();
}

double ArmSubsystem::GetAxis2Current() {
  return m_axis2Motor.GetOutputCurrent();
}

void ArmSubsystem::SetAxis1DutyCycle(double dutyCycle) {
  m_axis1PIDController.SetReference(dutyCycle, rev::CANSparkMax::ControlType::kDutyCycle);
}

void ArmSubsystem::SetAxis2DutyCycle(double dutyCycle) {
  m_axis2PIDController.SetReference(dutyCycle, rev::CANSparkMax::ControlType::kDutyCycle);
}

void ArmSubsystem::SetAxis1Off() {
  m_axis1Motor.StopMotor();
  frc::DataLogManager::Log(fmt::format("Axis1: Disabled motor by commanding voltage to 0"));
}

void ArmSubsystem::SetAxis2Off() {
  m_axis2Motor.StopMotor();
  frc::DataLogManager::Log(fmt::format("Axis2: Disabled motor by commanding voltage to 0."));
}

void ArmSubsystem::SetArmOff() {
  ArmSubsystem::SetAxis1Off();
  ArmSubsystem::SetAxis2Off();
}

void ArmSubsystem::ZeroAxis1() {
  m_axis1Encoder.SetPosition(0);
  m_axis1PIDController.SetReference(0, rev::CANSparkMax::ControlType::kPosition);
  frc::DataLogManager::Log(fmt::format("Axis1: Zeroed and started position pid at 0."));
}

void ArmSubsystem::ZeroAxis2() {
  m_axis2Encoder.SetPosition(0);
  m_axis2PIDController.SetReference(0, rev::CANSparkMax::ControlType::kPosition);
  frc::DataLogManager::Log(fmt::format("Axis2: Zeroed and started position pid at 0."));
}

void ArmSubsystem::Move(units::degree_t angle) {
  // Ensure we aren't commanding the arm outside it's limit.
  if (angle > kAxis1MaxAngle) {
    m_axis1SetPoint = kAxis1MaxAngle;
  } else if (angle < units::degree_t(0)) {
    m_axis1SetPoint = units::degree_t(0);
  } else {
    m_axis1SetPoint = angle;
  }

  m_isAxis1ControlEnabled = true;

  // Reset our initial profile states.
  m_axis1TrapezoidalProfileState =
    frc::TrapezoidProfile<units::degree>::State { ArmSubsystem::GetAxis1Angle(),
                                                  ArmSubsystem::GetAxis1Velocity() };
  frc::DataLogManager::Log(
    fmt::format("Arm: Move(Axis1) - Axis1SetPoint={}", m_axis1SetPoint.value()));
}

void ArmSubsystem::Move(units::inch_t distance) {
  // Ensure we aren't commanding the arm outside it's limit.
  if (distance > kAxis2MaxExtension) {
    m_axis2SetPoint = kAxis2MaxExtension;
  } else if (distance < units::inch_t(0)) {
    m_axis2SetPoint = units::inch_t(0);
  } else {
    m_axis2SetPoint = distance;
  }

  m_isAxis2ControlEnabled = true;

  // Reset our initial profile states.
  m_axis2TrapezoidalProfileState =
    frc::TrapezoidProfile<units::meters>::State { ArmSubsystem::GetAxis2Distance(),
                                                  ArmSubsystem::GetAxis2Velocity() };
  frc::DataLogManager::Log(
    fmt::format("Arm: Move(Axis2) - Axis2SetPoint={}", m_axis2SetPoint.value()));
}

void ArmSubsystem::Move(units::degree_t angle, units::inch_t distance) {
  ArmSubsystem::Move(angle);
  ArmSubsystem::Move(distance);
}

void ArmSubsystem::Move(ArmPosition position) {
  double axis1 = 0;
  double axis2 = 0;

  switch (position) {
    case STOWED:
      axis1 = 0;
      axis2 = 0;
      break;

    case INTAKE_FLOOR_CONE:
      axis1 = GetConeAxis1IntakeFloor();
      axis2 = GetConeAxis2IntakeFloor();
      break;

    case INTAKE_FLOOR_CUBE:
      axis1 = GetCubeAxis1IntakeFloor();
      axis2 = GetCubeAxis2IntakeFloor();
      break;

    case INTAKE_STATION_CONE:
      axis1 = GetConeAxis1IntakeStation();
      axis2 = GetConeAxis2IntakeStation();
      break;

    case INTAKE_STATION_CUBE:
      axis1 = GetCubeAxis1IntakeStation();
      axis2 = GetCubeAxis2IntakeStation();
      break;

    case SCORE_LOW_CONE:
      axis1 = GetConeAxis1ScoreLow();
      axis2 = GetConeAxis2ScoreLow();
      break;

    case SCORE_LOW_CUBE:
      axis1 = GetCubeAxis1ScoreLow();
      axis2 = GetCubeAxis2ScoreLow();
      break;

    case SCORE_MID_CONE:
      axis1 = GetConeAxis1ScoreMid();
      axis2 = GetConeAxis2ScoreMid();
      break;

    case SCORE_MID_CUBE:
      axis1 = GetCubeAxis1ScoreMid();
      axis2 = GetCubeAxis2ScoreMid();
      break;

    case SCORE_HIGH_CONE:
      axis1 = GetConeAxis1ScoreHigh();
      axis2 = GetConeAxis2ScoreHigh();
      break;

    case SCORE_HIGH_CUBE:
      axis1 = GetCubeAxis1ScoreHigh();
      axis2 = GetCubeAxis2ScoreHigh();
      break;
  }

  Move(units::degree_t(axis1), units::inch_t(axis2));
}

bool ArmSubsystem::IsAtCommandedPosition() {
  double axis1 = GetAxis1Angle().value();
  double axis2 = GetAxis2Distance().value();
  double axis1SetPoint = m_axis1SetPoint.value();
  double axis2SetPoint = m_axis2SetPoint.value();

  if (
    (axis1 < axis1SetPoint + kAxis1Deadband && axis1 > axis1SetPoint - kAxis1Deadband) &&
    (axis2 < axis2SetPoint + kAxis2Deadband && axis2 > axis2SetPoint - kAxis2Deadband)) {
    return true;
  }

  return false;
}

void ArmSubsystem::DisableControlAxis2() {
  m_isAxis2ControlEnabled = false;
}

frc2::CommandPtr ArmSubsystem::JogUp() {
  return frc2::FunctionalCommand(
           // Initialize
           [this] {
             ArmSubsystem::Move(
               GetAxis1Angle() + units::degree_t(GetJogDistance()), GetAxis2Distance());
           },
           // Execute
           [this] {},
           // End
           [this](bool interrupted) {},
           // IsFinished
           [this] { return ArmSubsystem::IsAtCommandedPosition(); },
           // Require subsystem
           { this })
    .ToPtr();
}

frc2::CommandPtr ArmSubsystem::JogDown() {
  return frc2::FunctionalCommand(
           // Initialize
           [this] {
             ArmSubsystem::Move(
               GetAxis1Angle() - units::degree_t(GetJogDistance()), GetAxis2Distance());
           },
           // Execute
           [this] {},
           // End
           [this](bool interrupted) {},
           // IsFinished
           [this] { return ArmSubsystem::IsAtCommandedPosition(); },
           // Require subsystem
           { this })
    .ToPtr();
}

bool ArmSubsystem::IsArmUp() {
  return GetAxis1Angle() > m_armUpThreshold;
}

units::meters_per_second_t ArmSubsystem::GetMaxSpeedOverride() const {
  return units::meters_per_second_t { GetDriveMaxSpeedOverride() };
}