
#include "commands/IntakeAutonomouslyCommand.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ScheduleCommand.h>
#include <frc2/command/WaitCommand.h>

#include "commands/ArmMoveCommand.h"
#include "commands/DriveToPoseCommand.h"
#include "commands/IntakeConeCommand.h"
#include "commands/IntakeCubeCommand.h"
#include "commands/PreIntakeLineupCommand.h"

IntakeAutonomouslyCommand::IntakeAutonomouslyCommand(
  DriveSubsystem* drive, ArmSubsystem* arm, IntakeSubsystem* intake, std::function<bool()> isCone) {
  AddCommands(
    // TODO: A turn command might be nice here?
    frc2::ParallelCommandGroup(
      frc2::ConditionalCommand(
        ArmMoveCommand(arm, ArmPosition::INTAKE_STATION_CONE),
        ArmMoveCommand(arm, ArmPosition::INTAKE_STATION_CONE),
        isCone),
      PreIntakeLineupCommand()),
    frc2::ParallelRaceGroup(IntakeConeCommand(intake)  //,DriveToPoseCommand()
                            ));
}
