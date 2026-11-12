#include "Robot.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {
  // Initializes recording of log data to disk and enables logging of
  // DS control and joystick data
  frc::DataLogManager::Start();
  auto ntinst = nt::NetworkTableInstance::GetDefault();
  m_bpIsLogging = ntinst.GetBooleanTopic("/Vision/IsLogging").Publish();
  m_bpIsLogging.Set(false);

  m_cameraIsWritingLog = ntinst.GetBooleanTopic("/Vision/PiIsLogging").Subscribe(false);

#ifdef LOG_DATA_DRIVER_STATION
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
#endif
}

/**
 * This function is called every 20 ms, no matter the mode. Use this for items
 * like diagnostics that you want to run during disabled, autonomous, teleop and test.
 *
 * This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  frc::SmartDashboard::PutString("RobotState", m_container.GetRobotModeString());
}

/**
 * This function is called once each time the robot enters Disabled mode.
 * You can use it to reset any subsystem information you want to clear when the robot is disabled.
 */
void Robot::DisabledInit() {
  m_container.OnDisable();
  m_writingStarted = false;
}

void Robot::DisabledPeriodic() {
  if (m_cameraIsWritingLog.Get() && !m_writingStarted) {
    m_writingStarted = true;
    m_container.SetLedsCameraWriting();
    frc::DataLogManager::Log("Pi is logging match images.");
  } else if (m_writingStarted && !m_cameraIsWritingLog.Get()) {
    m_writingStarted = false;
    m_bpIsLogging.Set(false);
    m_container.SetLedsCameraDoneWriting();
    frc::DataLogManager::Log("Pi is done logging match images.");
  }
}

/**
 * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_container.OnAutoEnable();
  StartCamera();

  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when teleop starts running.
  // If you want the autonomous to continue until interrupted by another command,
  // remove this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }

  m_container.OnTeleEnable();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

void Robot::StartCamera() {
  frc::DataLogManager::Log(
    "Setting Vision/IsLogging to true. Camera will now log images to memory.");
  m_bpIsLogging.Set(true);
}
