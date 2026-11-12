// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CompressorConfigType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.CompressorConstants;
import frc.robot.Constants.RobotConstants;

public class Robot extends TimedRobot{
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  // Construct our robot and set the cycle time to be 10ms.
  public Robot() {
    super(RobotConstants.kRobotPeriod);
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    if(SmartDashboard.getBoolean("Compressor/EnabledState", false)) {
      m_compressor.enableDigital();
    } else {
      m_compressor.disable();
    }

    // Put commands on the dashboard to enable and disable the compressor.
    SmartDashboard.putData("Compressor/Disable", 
      new InstantCommand(() -> m_compressor.disable()));
    SmartDashboard.putData("Compressor/Enable", 
      new InstantCommand(() -> m_compressor.enableDigital()));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    // Report information to the dashboard.
    SmartDashboard.putBoolean("Compressor/EnabledState", 
      m_compressor.getConfigType() != CompressorConfigType.Disabled);
    SmartDashboard.putNumber("Compressor/Pressure", m_compressor.getPressure());
    SmartDashboard.putNumber("Compressor/Current", m_compressor.getCurrent());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
