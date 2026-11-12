#include "subsystems/MAXSwerveModule.h"

#include <frc/geometry/Rotation2d.h>

#include <numbers>

#include "Constants.h"
#include "Settings.h"
#include "utils/MathUtils.h"

using namespace ModuleConstants;

// clang-format off
REGISTER_DOUBLE(DriveTuneModuleDrivingP, "Drive/Tune/Module/Driving - P", ModuleConstants::kDrivingP);
REGISTER_DOUBLE(DriveTuneModuleDrivingI, "Drive/Tune/Module/Driving - I", ModuleConstants::kDrivingI);
REGISTER_DOUBLE(DriveTuneModuleDrivingD, "Drive/Tune/Module/Driving - D", ModuleConstants::kDrivingD);
REGISTER_DOUBLE(DriveTuneModuleDrivingFF, "Drive/Tune/Module/Driving - FF", ModuleConstants::kDrivingFF);
REGISTER_DOUBLE(DriveTuneModuleTurningP, "Drive/Tune/Module/Turning - P", ModuleConstants::kTurningP);
REGISTER_DOUBLE(DriveTuneModuleTurningI, "Drive/Tune/Module/Turning - I", ModuleConstants::kTurningI);
REGISTER_DOUBLE(DriveTuneModuleTurningD, "Drive/Tune/Module/Turning - D", ModuleConstants::kTurningD);
REGISTER_DOUBLE(DriveTuneModuleTurningFF, "Drive/Tune/Module/Turning - FF", ModuleConstants::kTurningFF);

// clang-format on

MAXSwerveModule::MAXSwerveModule(
  const int drivingCANId, const int turningCANId, const units::radian_t chassisAngularOffset) :
  m_drivingSparkMax(drivingCANId, rev::CANSparkMax::MotorType::kBrushless),
  m_turningSparkMax(turningCANId, rev::CANSparkMax::MotorType::kBrushless) {
  // Factory reset, so we get the SPARK MAX to a known state before configuring them.
  // This is useful in case a SPARK MAX is swapped out.
  m_drivingSparkMax.RestoreFactoryDefaults();
  m_turningSparkMax.RestoreFactoryDefaults();

  // Apply position and velocity conversion factors for the driving encoder.
  // The native units for position and velocity are rotations and RPM, respectively,
  // but we want meters and meters per second to use with WPILib's swerve APIs.
  m_drivingEncoder.SetPositionConversionFactor(kDrivingEncoderPositionFactor);
  m_drivingEncoder.SetVelocityConversionFactor(kDrivingEncoderVelocityFactor);

  // These parameters control the behavior of the velocity filter on the SPARK MAX
  // Default settings result in about 100+ms delay due to multiple samples being
  // taken to filter out noise in the velocity readings. Reducing depth results
  // in higher noise in data, but reduces the lag.
  m_drivingEncoder.SetMeasurementPeriod(kDrivingVelocityMeasurementPeriodMs);
  m_drivingEncoder.SetAverageDepth(kDrivingVelocityMeasurementDepth);

  // Apply position and velocity conversion factors for the turning encoder.
  // We want these in radians and radians per second to use with WPILib's swerve APIs.
  m_turningAbsoluteEncoder.SetPositionConversionFactor(kTurningEncoderPositionFactor);
  m_turningAbsoluteEncoder.SetVelocityConversionFactor(kTurningEncoderVelocityFactor);

  // Invert the turning encoder, since the output shaft rotates in the opposite
  // direction of the steering motor in the MAXSwerve Module.
  m_turningAbsoluteEncoder.SetInverted(kTurningEncoderInverted);

  // Enable PID wrap around for the turning motor. This will allow the PID controller
  // to go through 0 to get to the setpoint i.e. going from 350 degrees to 10 degrees
  // will go through 0 rather than the other direction which is a longer route.
  m_turningPIDController.SetPositionPIDWrappingEnabled(true);
  m_turningPIDController.SetPositionPIDWrappingMinInput(kTurningEncoderPositionPIDMinInput.value());
  m_turningPIDController.SetPositionPIDWrappingMaxInput(kTurningEncoderPositionPIDMaxInput.value());

  // Set the PID Controller to use the duty cycle encoder on the swerve
  // module instead of the built in NEO550 encoder.
  m_turningPIDController.SetFeedbackDevice(m_turningAbsoluteEncoder);

  // Set the PID gains for the driving motor.
  // These gains need to be tuned for the specific robot!
  m_drivingPIDController.SetP(GetDriveTuneModuleDrivingP());
  m_drivingPIDController.SetI(GetDriveTuneModuleDrivingI());
  m_drivingPIDController.SetD(GetDriveTuneModuleDrivingD());
  m_drivingPIDController.SetFF(GetDriveTuneModuleDrivingFF());
  m_drivingPIDController.SetOutputRange(kDrivingMinOutput, kDrivingMaxOutput);

  // Set the PID gains for the turning motor.
  // These gains need to be tuned for the specific robot!
  m_turningPIDController.SetP(GetDriveTuneModuleTurningP());
  m_turningPIDController.SetI(GetDriveTuneModuleTurningI());
  m_turningPIDController.SetD(GetDriveTuneModuleTurningD());
  m_turningPIDController.SetFF(GetDriveTuneModuleTurningFF());
  m_turningPIDController.SetOutputRange(kTurningMinOutput, kTurningMaxOutput);

  m_drivingSparkMax.SetIdleMode(kDrivingMotorIdleMode);
  m_turningSparkMax.SetIdleMode(kTurningMotorIdleMode);
  m_drivingSparkMax.SetSmartCurrentLimit(kDrivingMotorCurrentLimit.value());
  m_turningSparkMax.SetSmartCurrentLimit(kTurningMotorCurrentLimit.value());
  m_drivingSparkMax.EnableVoltageCompensation(kVoltageCompensation.value());
  m_turningSparkMax.EnableVoltageCompensation(kVoltageCompensation.value());

  // Save the SPARK MAX configurations. If a SPARK MAX browns out during
  // operation, it will maintain the above configurations.
  m_drivingSparkMax.BurnFlash();
  m_turningSparkMax.BurnFlash();

  m_chassisAngularOffset = chassisAngularOffset;
  m_desiredState.angle =
    frc::Rotation2d(units::radian_t { m_turningAbsoluteEncoder.GetPosition() });

  m_drivingEncoder.SetPosition(0);
}

SwerveModuleState MAXSwerveModule::GetState() const {
  return {
    units::meters_per_second_t { m_drivingEncoder.GetVelocity() },
    units::radian_t { m_turningAbsoluteEncoder.GetPosition() } - m_chassisAngularOffset,
  };
}

frc::SwerveModulePosition MAXSwerveModule::GetPosition() const {
  return {
    units::meter_t { m_drivingEncoder.GetPosition() },
    units::radian_t { m_turningAbsoluteEncoder.GetPosition() } - m_chassisAngularOffset,
  };
}

double MAXSwerveModule::GetDrivingCurrent() {
  return m_drivingSparkMax.GetOutputCurrent();
}

double MAXSwerveModule::GetTurningCurrent() {
  return m_turningSparkMax.GetOutputCurrent();
}

units::meters_per_second_t MAXSwerveModule::GetDesiredSpeed() const {
  return m_desiredState.speed;
}

units::radian_t MAXSwerveModule::GetDesiredAngle() const {
  return m_desiredState.angle.Radians() - m_chassisAngularOffset;
}

void MAXSwerveModule::SetDesiredState(const SwerveModuleState& desiredState) {
  // apply chassis angular offset to the desired state
  SwerveModuleState correctedDesiredState {};
  correctedDesiredState.speed = desiredState.speed;
  correctedDesiredState.angle = desiredState.angle + frc::Rotation2d(m_chassisAngularOffset);

  // optimize the reference state to avoid spinning further than 90 degrees
  const auto currentAngle = units::radian_t { m_turningAbsoluteEncoder.GetPosition() };
  const auto optimizedDesiredState = Optimize(correctedDesiredState, currentAngle);

  const auto desiredSpeed = optimizedDesiredState.speed.value();            // meters per second
  const auto desiredAngle = optimizedDesiredState.angle.Radians().value();  // angle

#ifdef SWERVE_SECOND_ORDER_KINEMATICS
  const auto desiredOmega = optimizedDesiredState.omega;  // radians per second
  const auto turningFF = kVoltageNominal.value() * desiredOmega / kTurningMotorMaxAngularVelocity;
#endif

  // command driving and turning SPARK MAX towards their respective setpoints
  m_drivingPIDController.SetReference(desiredSpeed, rev::CANSparkMax::ControlType::kVelocity);

#ifdef SWERVE_SECOND_ORDER_KINEMATICS
  m_turningPIDController.SetReference(
    desiredAngle, rev::CANSparkMax::ControlType::kPosition, 0, turningFF);
#else
  m_turningPIDController.SetReference(desiredAngle, rev::CANSparkMax::ControlType::kPosition;
#endif

  m_desiredState = optimizedDesiredState;
}

SwerveModuleState MAXSwerveModule::Optimize(
  const SwerveModuleState& desiredState, units::radian_t currentAngle) {
  // do NOT use FRC version since it might suffer from numerical instability
  const auto delta =
    fabs(MathUtils::ClampAngle(desiredState.angle.Radians() - currentAngle).value());

  if (MathUtils::IsZero(delta, 0.1)) {
    return desiredState;
  }

  constexpr auto threshold =
    std::numbers::pi * 0.5 * 1.01;  // a bit over 90 degrees to avoid instability

  if (delta > threshold) {
    return SwerveModuleState {
      -desiredState.speed,
      frc::Rotation2d(desiredState.angle.Degrees() + 180_deg),
    };
  }

  return desiredState;
}

void MAXSwerveModule::ResetEncoders() {
  m_drivingEncoder.SetPosition(0);
}

void MAXSwerveModule::UpdateTuningParameters() {
  m_drivingPIDController.SetP(GetDriveTuneModuleDrivingP());
  m_drivingPIDController.SetI(GetDriveTuneModuleDrivingI());
  m_drivingPIDController.SetD(GetDriveTuneModuleDrivingD());
  m_drivingPIDController.SetFF(GetDriveTuneModuleDrivingFF());

  m_turningPIDController.SetP(GetDriveTuneModuleTurningP());
  m_turningPIDController.SetI(GetDriveTuneModuleTurningI());
  m_turningPIDController.SetD(GetDriveTuneModuleTurningD());
  m_turningPIDController.SetFF(GetDriveTuneModuleTurningFF());
}
