#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <networktables/BooleanTopic.h>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc2::Command* m_autonomousCommand = nullptr;
  nt::BooleanPublisher m_bpIsLogging;
  nt::BooleanSubscriber m_cameraIsWritingLog;

  RobotContainer m_container;
  void StartCamera();
  bool m_writingStarted = false;
};
