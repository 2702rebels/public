// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {
  // Subsystems
  private DrivetrainSubsystem m_drive = new DrivetrainSubsystem();

  // The driver's controller
  private CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_drive.setDefaultCommand(
      m_drive.arcadeDriveCommand(
        () -> -m_driverController.getLeftY(), () -> m_driverController.getRightX()));

    m_driverController.rightStick().onTrue(new InstantCommand(() -> m_drive.toggleGearbox()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
