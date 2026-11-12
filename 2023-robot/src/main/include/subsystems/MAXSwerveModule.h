#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxAbsoluteEncoder.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/SparkMaxRelativeEncoder.h>

#include "Constants.h"
#include "utils/SwerveModuleState2.h"

#ifdef SWERVE_SECOND_ORDER_KINEMATICS
using SwerveModuleState = SwerveModuleState2;
#else
using SwerveModuleState = frc::SwerveModuleState;
#endif

class MAXSwerveModule {
 public:
  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore Encoder.
   */
  MAXSwerveModule(
    const int driveCANId, const int turningCANId, const units::radian_t chassisAngularOffset);

  /**
   * Returns the current state of the module.
   * @return The current state of the module.
   */
  SwerveModuleState GetState() const;

  /**
   * Returns the current position of the module.
   * @return The current position of the module.
   */
  frc::SwerveModulePosition GetPosition() const;

  /**
   * Sets the desired state for the module.
   * @param desiredState Desired state with speed and angle.
   */
  void SetDesiredState(const SwerveModuleState& state);

  /**
   * Gets the desired speed.
   */
  units::meters_per_second_t GetDesiredSpeed() const;

  /**
   * Gets the desired angle.
   */
  units::radian_t GetDesiredAngle() const;

  /**
   * Gets the current of the driving motor.
   */
  double GetDrivingCurrent();

  /**
   * Gets the current of the turning motor.
   */
  double GetTurningCurrent();

  /**
   * Zeroes all the SwerveModule encoders.
   */
  void ResetEncoders();

  /**
   * Re-reads and re-assigns various tuning parameters.
   */
  void UpdateTuningParameters();

  /**
   * Optimize the reference state to avoid spinning further than 90 degrees.
   */
  SwerveModuleState Optimize(const SwerveModuleState& desiredState, units::radian_t currentAngle);

 private:
  rev::CANSparkMax m_drivingSparkMax;
  rev::CANSparkMax m_turningSparkMax;

  rev::SparkMaxRelativeEncoder m_drivingEncoder = m_drivingSparkMax.GetEncoder();
  rev::SparkMaxAbsoluteEncoder m_turningAbsoluteEncoder =
    m_turningSparkMax.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);

  rev::SparkMaxPIDController m_drivingPIDController = m_drivingSparkMax.GetPIDController();
  rev::SparkMaxPIDController m_turningPIDController = m_turningSparkMax.GetPIDController();

  units::radian_t m_chassisAngularOffset { 0_rad };
  SwerveModuleState m_desiredState { units::meters_per_second_t { 0.0 }, frc::Rotation2d() };
};
