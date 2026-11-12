
#include "commands/IntakeClapCommand.h"

IntakeClapCommand::IntakeClapCommand(IntakeSubsystem* m_intake) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void IntakeClapCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IntakeClapCommand::Execute() {}

// Called once the command ends or is interrupted.
void IntakeClapCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool IntakeClapCommand::IsFinished() {
  return false;
}
