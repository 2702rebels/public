
#include "commands/ScoreAutonomouslyCommand.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ScheduleCommand.h>
#include <frc2/command/WaitCommand.h>

#include "commands/ArmMoveCommand.h"
#include "commands/DriveToPoseCommand.h"
#include "commands/PreScoreLineUpCommand.h"
#include "commands/ScoreConeCommand.h"
#include "commands/ScoreCubeCommand.h"

ScoreAutonomouslyCommand::ScoreAutonomouslyCommand(
  DriveSubsystem* drive,
  ArmSubsystem* arm,
  IntakeSubsystem* intake,
  std::function<bool()> isHigh,
  std::function<bool()> isCone) {
  AddCommands(
    frc2::ParallelCommandGroup(
      frc2::ConditionalCommand(
        frc2::ConditionalCommand(
          ArmMoveCommand(arm, ArmPosition::SCORE_HIGH_CONE),
          ArmMoveCommand(arm, ArmPosition::SCORE_MID_CONE),
          isHigh),
        frc2::ConditionalCommand(
          ArmMoveCommand(arm, ArmPosition::SCORE_HIGH_CUBE),
          ArmMoveCommand(arm, ArmPosition::SCORE_MID_CUBE),
          isHigh),
        isCone),
      PreScoreLineUpCommand()),
    // DriveToPoseCommand())),
    frc2::ConditionalCommand(ScoreConeCommand(arm, intake), ScoreCubeCommand(arm, intake), isCone));
}
