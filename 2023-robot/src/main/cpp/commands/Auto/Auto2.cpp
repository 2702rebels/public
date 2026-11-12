#include "commands/auto/Auto2.h"

#include "Field.h"
#include "commands/AutoBalanceCommand.h"
#include "commands/DriveRotateCommand.h"
#include "commands/DriveToPoseCommand.h"
#include "commands/DriveTrajectoryCommand.h"
#include "frc2/command/InstantCommand.h"

constexpr auto offset = frc::Translation2d { RobotConstants::kBumperSize / 2, 0_m };

Auto2::Auto2(DriveSubsystem* drive, LedSubsystem* led) {
  AddCommands(
    frc2::InstantCommand([led] { led->SetFrontLEDsOn(false); }, {}),
    frc2::InstantCommand([drive] { drive->ResetOdometry(Field::GetFieldPose(Field::CONE_6)); }, {}),
    DriveRotateCommand(1_deg, drive),
    DriveToPoseCommand({ Field::GetGamePiece(3) - offset, 0_deg }, drive, true),
    DriveToPoseCommand(Field::GetFieldPose(Field::CUBE_3), drive, true));
}
