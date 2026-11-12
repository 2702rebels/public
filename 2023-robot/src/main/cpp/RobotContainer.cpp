#include "RobotContainer.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ScheduleCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>

#include <utility>

#include "Constants.h"
#include "commands/ArmMoveCommand.h"
#include "commands/DriveAlignCommand.h"
#include "commands/DriveToPoseCommand.h"
#include "commands/IntakeAutonomouslyCommand.h"
#include "commands/IntakeClapCommand.h"
#include "commands/IntakeConeCommand.h"
#include "commands/IntakeCubeCommand.h"
#include "commands/IntakeHomeGripperCommand.h"
#include "commands/ScoreAutonomouslyCommand.h"
#include "commands/ScoreConeCommand.h"
#include "commands/ScoreCubeCommand.h"
#include "commands/SetLedCommand.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // initialize all of your commands and subsystems here

#ifdef LED_ENABLED
  m_leds.SetRGB(255, 0, 0);
  m_leds.SetPattern(LEDPattern::BREATH, 1_s);
#endif

// register subsystems on the shuffleboard
#ifdef DRIVETRAIN_ENABLED
  frc::Shuffleboard::GetTab("Drivetrain").Add(m_drive);
#endif
#ifdef ARM_ENABLED
  frc::Shuffleboard::GetTab("Arm").Add(m_arm);
#endif
#ifdef INTAKE_ENABLED
  frc::Shuffleboard::GetTab("Intake").Add(m_intake);
#endif

#ifdef TRACE_LOG
  // Log Shuffleboard events for command initialize, execute, finish, interrupt
  frc2::CommandScheduler::GetInstance().OnCommandInitialize([](const frc2::Command& command) {
    std::string_view commandName = command.GetName();
    frc::Shuffleboard::AddEventMarker(
      "Command initialized", commandName, frc::ShuffleboardEventImportance::kNormal);
    if (commandName != "CommandBase") {
      frc::DataLogManager::Log(fmt::format("Scheduler|{}: Initialize", commandName));
    }
  });
  // frc2::CommandScheduler::GetInstance().OnCommandExecute([](const frc2::Command& command) {
  //   frc::Shuffleboard::AddEventMarker(
  //     "Command executed", command.GetName(), frc::ShuffleboardEventImportance::kNormal);
  //   // frc::DataLogManager::Log(fmt::format("Scheduler|{}: Execute", command.GetName()));
  // });
  frc2::CommandScheduler::GetInstance().OnCommandFinish([](const frc2::Command& command) {
    std::string_view commandName = command.GetName();
    frc::Shuffleboard::AddEventMarker(
      "Command finished", commandName, frc::ShuffleboardEventImportance::kNormal);
    if (commandName != "CommandBase") {
      frc::DataLogManager::Log(fmt::format("Scheduler|{}: Finish", commandName));
    }
  });
  frc2::CommandScheduler::GetInstance().OnCommandInterrupt([](const frc2::Command& command) {
    std::string_view commandName = command.GetName();
    frc::Shuffleboard::AddEventMarker(
      "Command interrupted", commandName, frc::ShuffleboardEventImportance::kNormal);
    if (commandName != "CommandBase") {
      frc::DataLogManager::Log(fmt::format("Scheduler|{}: Interupt", commandName));
    }
  });
#endif

  // configure the button bindings
  ConfigureButtonBindings();

  // set up default drive command:
  // - the left stick controls translation of the robot
  // - turning is controlled by the X axis of the right stick
#ifdef DRIVETRAIN_ENABLED
  m_drive.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_drive.Drive(
        -frc::ApplyDeadband(m_driverController.GetLeftY(), m_drive.GetDeadband()),
        -frc::ApplyDeadband(m_driverController.GetLeftX(), m_drive.GetDeadband()),
        -frc::ApplyDeadband(m_driverController.GetRightX(), m_drive.GetDeadband()),
        true);
    },
    { &m_drive }));
#endif

  // autonomous command chooser
#ifdef AUTO_CHOOSER
  frc::ShuffleboardTab& tab = frc::Shuffleboard::GetTab("Auto");
  m_chooser.SetDefaultOption("DoNothing", &m_auto_doNothing);
  m_chooser.AddOption("DriveForwards", &m_auto_driveForwards);
  m_chooser.AddOption("Auto2", &m_auto2);
  m_chooser.AddOption("Auto3", &m_auto3);
#ifdef ARM_ENABLED
  m_chooser.AddOption("ScoreAndOverBalance", &m_auto_scoreAndOverAndBalance);
  m_chooser.AddOption("ScoreAndOut (Red)", &m_auto_scoreAndOutRed);
  m_chooser.AddOption("ScoreAndBalance (Red)", &m_auto_scoreAndBalanceRed);
  m_chooser.AddOption("ScoreAndOut (Blue)", &m_auto_scoreAndOutBlue);
  m_chooser.AddOption("ScoreAndBalance (Blue)", &m_auto_scoreAndBalanceBlue);
  m_chooser.AddOption("ScoreScoreSco", &m_auto_scoreScoreSco);
#endif
  tab.Add(m_chooser);
#endif
}

void RobotContainer::OnAutoEnable() {
#ifdef LED_ENABLED
  // m_leds.SetRGB(0, 255, 0);
  // m_leds.SetPattern(LEDPattern::FLASH, 500_ms);
  frc2::CommandScheduler::GetInstance().Schedule(
    SetLedCommand(&m_leds, 0, 255, 0, LEDPattern::LOADING, 15_s, 15_s)
      .ToPtr()
      .AndThen(SetLedCommand(&m_leds, 255, 255, 255, LEDPattern::FLASH, 3_s).ToPtr()));
#endif
}

void RobotContainer::OnTeleEnable() {
  SetRobotMode(RobotMode::CUBE);
#ifdef INTAKE_ENABLED
  m_intake.CloseCube();
  m_intake.HoldGamePiece();
  SetRobotScoreHeight(RobotScoreHeight::HIGH);
#endif
}

void RobotContainer::OnDisable() {
#ifdef ARM_ENABLED
  m_arm.OnDisable();
#endif

#ifdef INTAKE_ENABLED
  m_intake.OnDisable();
#endif

#ifdef LED_ENABLED
  m_leds.SetRGB(255, 0, 0);
  m_leds.SetPattern(LEDPattern::BREATH, 3_s);
#endif
}

void RobotContainer::SetRobotMode(RobotMode mode) {
  m_mode = mode;
  if (m_mode == RobotMode::CUBE) {
#ifdef LED_ENABLED
    m_leds.SetRGB(255, 0, 255);
    m_leds.SetPattern(LEDPattern::SOLID, 0_s);
    frc::DataLogManager::Log("RobotMode set to CUBE.");
#endif
  } else if (m_mode == RobotMode::CONE) {
#ifdef LED_ENABLED
    m_leds.SetRGB(255, 255, 0);
    m_leds.SetPattern(LEDPattern::SOLID, 0_s);
    frc::DataLogManager::Log("RobotMode set to CONE.");
#endif
  }
}

void RobotContainer::ToggleRobotMode() {
  if (m_mode == RobotMode::CONE) {
    SetRobotMode(RobotMode::CUBE);
  } else {
    SetRobotMode(RobotMode::CONE);
  }
}

std::string RobotContainer::GetRobotModeString() {
  if (m_mode == RobotMode::CONE) {
    return "CONE";
  } else if (m_mode == RobotMode::CUBE) {
    return "CUBE";
  }
  return "Undefined";
}

void RobotContainer::SetRobotScoreHeight(RobotScoreHeight height) {
  m_scoreHeight = height;
}

void RobotContainer::ToggleRobotScoreHeight() {
  if (m_scoreHeight == RobotScoreHeight::HIGH) {
    SetRobotScoreHeight(RobotScoreHeight::MID);
  } else {
    SetRobotScoreHeight(RobotScoreHeight::HIGH);
  }
}

std::string RobotContainer::GetRobotScoreHeightString() {
  if (m_scoreHeight == RobotScoreHeight::HIGH) {
    return "HIGH";
  } else if (m_scoreHeight == RobotScoreHeight::MID) {
    return "MID";
  }
  return "Undefined";
}

void RobotContainer::ToggleFrontLights() {
  if (m_isFrontLights) {
    m_isFrontLights = false;
    m_leds.SetFrontLEDsOn(false);
    frc::DataLogManager::Log("Set front lights off.");
  } else {
    m_isFrontLights = true;
    m_leds.SetFrontLEDsOn(true);
    frc::DataLogManager::Log("Set front lights on.");
  }
}

void RobotContainer::ConfigureButtonBindings() {
  // [Configure Driver Controller]

#ifdef DRIVETRAIN_ENABLED
  // reset swerve odometry to the currently selected scoring station
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightStick)
    .OnTrue(frc2::InstantCommand(
              [this] { m_drive.ResetOdometry(m_field.GetScorePosition()); }, { &m_drive })
              .ToPtr());

  // Lock the wheels
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kStart)
    .WhileTrue(frc2::RunCommand([this] { m_drive.StopX(); }, { &m_drive }).ToPtr());

  frc2::Trigger(
    m_driverController.POVLeft(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(
      frc2::InstantCommand([this] { m_drive.UpdateTuningParameters(); }, { &m_drive }).ToPtr());

  frc2::Trigger(
    m_driverController.LeftBumper(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(frc2::InstantCommand([this] { m_drive.ToggleOrientationLock(180_deg); }, { &m_drive })
              .ToPtr());

  frc2::Trigger(
    m_driverController.RightBumper(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(
      frc2::InstantCommand([this] { m_drive.ToggleOrientationLock(0_deg); }, { &m_drive }).ToPtr());

  // frc2::Trigger(
  //   m_driverController.POVDown(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
  //   .OnTrue(frc2::InstantCommand([this] { m_drive.ResetOdometry(frc::Pose2d {}); }, { &m_drive })
  //             .ToPtr());
#endif

#ifdef ARM_ENABLED

  // Jog Up.

  // frc2::Trigger(
  //   m_driverController.POVUp(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
  //   .OnTrue(m_arm.JogUp());

  // Jog Down.

  // frc2::Trigger(
  //   m_driverController.POVDown(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
  //   .OnTrue(m_arm.JogDown());

  // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA)
  //   .OnTrue(ArmMoveCommand(&m_arm, ArmPosition::STOWED).ToPtr());

  // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB)
  //   .OnTrue(frc2::cmd::Either(
  //     ArmMoveCommand(&m_arm, ArmPosition::SCORE_MID_CONE).ToPtr(),
  //     ArmMoveCommand(&m_arm, ArmPosition::SCORE_MID_CUBE).ToPtr(),
  //     [this] { return m_mode == RobotMode::CONE; }));

  // Score Low.
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA)
    .OnTrue(frc2::cmd::Sequence(
      ArmMoveCommand(&m_arm, ArmPosition::SCORE_LOW_CUBE).ToPtr(),
      frc2::InstantCommand([this] { m_intake.SetIntakeSpeed(-0.5); }, { &m_intake }).ToPtr()))
    .OnFalse(frc2::InstantCommand([this] { m_intake.SetIntakeSpeed(0.0); }, { &m_intake }).ToPtr());

#ifdef DRIVETRAIN_ENABLED
  frc2::Trigger([this] { return !frc::DriverStation::IsAutonomous() && m_arm.IsArmUp(); })
    .OnTrue(
      frc2::InstantCommand([this] { m_drive.SetMaxSpeedOverride(m_arm.GetMaxSpeedOverride()); }, {})
        .ToPtr())
    .OnFalse(
      frc2::InstantCommand([this] { m_drive.ResetMaxSpeedOverride(); }, { &m_drive }).ToPtr());
#endif

  // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kY)
  //   .OnTrue(frc2::cmd::Either(
  //     ArmMoveCommand(&m_arm, ArmPosition::SCORE_HIGH_CONE).ToPtr(),
  //     ArmMoveCommand(&m_arm, ArmPosition::SCORE_HIGH_CUBE).ToPtr(),
  //     [this] { return m_mode == RobotMode::CONE; }));

#ifdef INTAKE_ENABLED
  // Intake automatically.
  // frc2::Trigger(m_driverController.LeftTrigger(
  //                 0.5, frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
  //   .OnTrue(IntakeAutonomouslyCommand(&m_drive, &m_arm, &m_intake, [this]() -> bool {
  //             return this->m_mode == RobotMode::CONE;
  //           }).ToPtr());

  // Score automatically.
  // frc2::Trigger(m_driverController.RightTrigger(
  //                 0.5, frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
  //   .OnTrue(ScoreAutonomouslyCommand(
  //             &m_drive,
  //             &m_arm,
  //             &m_intake,
  //             [this]() -> bool { return this->m_scoreHeight == RobotScoreHeight::HIGH; },
  //             [this]() -> bool { return this->m_mode == RobotMode::CONE; })
  //             .ToPtr());

  // automatically drive to the scoring position while the trigger is held
  frc2::Trigger(m_driverController.RightTrigger(
                  0.5, frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .WhileTrue(DriveToPoseCommand([this] { return m_field.GetScorePosition(); }, &m_drive).ToPtr());

  // automatically align and drive to the loading position while the trigger is held
  // this works only if we have the corresponding april tag in view
  frc2::Trigger(m_driverController.LeftTrigger(
                  0.5, frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .WhileTrue(DriveAlignCommand(
                 [this] { return m_drive.GetLoadBeam(); },
                 [this] { return m_field.GetLoadOffset(); },
                 &m_drive)
                 .ToPtr());

  // Intake from the floor.
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX)
    .OnTrue(frc2::cmd::Either(
      frc2::cmd::Sequence(
        ArmMoveCommand(&m_arm, ArmPosition::INTAKE_FLOOR_CONE).ToPtr(),
        IntakeConeCommand(&m_intake).ToPtr(),
        SetLedCommand(&m_leds, 0, 0, 255, LEDPattern::FLASH, 300_ms, 3_s).ToPtr()),
      frc2::cmd::Sequence(
        ArmMoveCommand(&m_arm, ArmPosition::INTAKE_FLOOR_CUBE).ToPtr(),
        IntakeCubeCommand(&m_intake).ToPtr(),
        SetLedCommand(&m_leds, 0, 0, 255, LEDPattern::FLASH, 300_ms, 3_s).ToPtr()),
      [this] { return m_mode == RobotMode::CONE; }))
    .OnFalse(ArmMoveCommand(&m_arm, ArmPosition::STOWED).ToPtr());

  // Toggle robot mode.
  // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftBumper)
  //   .OnTrue(frc2::InstantCommand([this] { ToggleRobotMode(); }, {}).ToPtr());
#endif
#endif

#ifdef OPERATOR_CONTROLLER_ENABLED
  // [Configure Operator Controller]
  m_field.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_field.ShiftSelection(
        frc::ApplyDeadband(m_operatorController.GetLeftX(), m_drive.GetDeadband()),
        -frc::ApplyDeadband(m_operatorController.GetLeftY(), m_drive.GetDeadband()));
    },
    { &m_field }));

#ifdef ARM_ENABLED
#ifdef INTAKE_ENABLED
  // Reload parameters.
  frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kStart)
    .OnTrue(new frc2::InstantCommand(
      [this] {
        m_arm.ConfigureTuningParams();
        m_intake.ConfigureTuningParams();
      },
      {}));

  // Zero's the arm axis. TODO: Change this to run the homing command.
  frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kBack)
    .OnTrue(IntakeHomeGripperCommand(&m_intake).ToPtr());
#endif
#endif

#ifdef ARM_ENABLED

  // Jog Up.
  frc2::Trigger(
    m_operatorController.POVUp(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(m_arm.JogUp());

  // Jog Down.
  frc2::Trigger(
    m_operatorController.POVDown(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(m_arm.JogDown());

  // Stow.
  frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kA)
    .OnTrue(ArmMoveCommand(&m_arm, ArmPosition::STOWED).ToPtr());

  // Arm Mid.
  frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kB)
    .OnTrue(frc2::cmd::Either(
      ArmMoveCommand(&m_arm, ArmPosition::SCORE_MID_CONE).ToPtr(),
      ArmMoveCommand(&m_arm, ArmPosition::SCORE_MID_CUBE).ToPtr(),
      [this] { return m_mode == RobotMode::CONE; }));

  // Score Low.
  // frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kX);

  frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kY)
    .OnTrue(frc2::cmd::Either(
      ArmMoveCommand(&m_arm, ArmPosition::SCORE_HIGH_CONE).ToPtr(),
      ArmMoveCommand(&m_arm, ArmPosition::SCORE_HIGH_CUBE).ToPtr(),
      [this] { return m_mode == RobotMode::CONE; }));

#ifdef INTAKE_ENABLED
  //  Score Manually.
  frc2::Trigger(m_operatorController.RightTrigger(
                  0.5, frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(frc2::cmd::Either(
      ScoreConeCommand(&m_arm, &m_intake).ToPtr(),
      ScoreCubeCommand(&m_arm, &m_intake).ToPtr(),
      [this] { return m_mode == RobotMode::CONE; }));

  // Intake manually.
  frc2::Trigger(m_operatorController.LeftTrigger(
                  0.5, frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(frc2::cmd::Either(
      frc2::cmd::Sequence(
        ArmMoveCommand(&m_arm, ArmPosition::INTAKE_STATION_CONE).ToPtr(),
        IntakeConeCommand(&m_intake).ToPtr(),
        SetLedCommand(&m_leds, 0, 0, 255, LEDPattern::FLASH, 300_ms, 3_s).ToPtr()),
      frc2::cmd::Sequence(
        ArmMoveCommand(&m_arm, ArmPosition::INTAKE_STATION_CUBE).ToPtr(),
        IntakeCubeCommand(&m_intake).ToPtr(),
        SetLedCommand(&m_leds, 0, 0, 255, LEDPattern::FLASH, 300_ms, 3_s).ToPtr()),
      [this] { return m_mode == RobotMode::CONE; }))
    .OnFalse(frc2::cmd::Sequence(ArmMoveCommand(&m_arm, ArmPosition::STOWED).ToPtr()));

  // Intake Floor.
  frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kRightBumper)
    .OnTrue(frc2::cmd::Either(
      frc2::cmd::Sequence(
        ArmMoveCommand(&m_arm, ArmPosition::INTAKE_FLOOR_CONE).ToPtr(),
        IntakeConeCommand(&m_intake).ToPtr(),
        SetLedCommand(&m_leds, 0, 0, 255, LEDPattern::FLASH, 300_ms, 3_s).ToPtr()),
      frc2::cmd::Sequence(
        ArmMoveCommand(&m_arm, ArmPosition::INTAKE_FLOOR_CUBE).ToPtr(),
        IntakeCubeCommand(&m_intake).ToPtr(),
        SetLedCommand(&m_leds, 0, 0, 255, LEDPattern::FLASH, 300_ms, 3_s).ToPtr()),
      [this] { return m_mode == RobotMode::CONE; }))
    .OnFalse(ArmMoveCommand(&m_arm, ArmPosition::STOWED).ToPtr());
#endif
#endif

  // Flash to sign for a game piece.
  frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kX)
    .OnTrue(
      frc2::InstantCommand([this] { m_leds.SetPattern(LEDPattern::FLASH, 200_ms); }, {}).ToPtr())
    .OnFalse(
      frc2::InstantCommand([this] { m_leds.SetPattern(LEDPattern::SOLID, 0_ms); }, {}).ToPtr());

  // Toggle robot mode.
  frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kLeftBumper)
    .OnTrue(frc2::InstantCommand([this] { ToggleRobotMode(); }, {}).ToPtr());
  // Toggle Front LED.
  frc2::Trigger(
    m_operatorController.POVRight(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(frc2::InstantCommand([this] { RobotContainer::ToggleFrontLights(); }, {}).ToPtr());
#endif
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
#ifdef AUTO_CHOOSER
  return m_chooser.GetSelected();
#else
  return new frc2::InstantCommand([this] { frc::DataLogManager::Log("No autonomous code"); });
#endif
}

void RobotContainer::SetLedsCameraWriting() {
  m_leds.SetRGBLeft(255, 0, 0);
  m_leds.SetRGBRight(0, 0, 255);
  m_leds.SetPattern(LEDPattern::FLASH, 500_ms);
}

void RobotContainer::SetLedsCameraDoneWriting() {
  m_leds.SetPattern(LEDPattern::SOLID, 0_s);
  m_leds.SetRGB(0, 255, 0);
  m_leds.SetPattern(LEDPattern::BREATH, 2500_ms);
}
