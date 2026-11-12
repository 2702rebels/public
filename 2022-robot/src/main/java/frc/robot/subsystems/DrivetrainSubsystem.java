// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  private final CANSparkMax m_leftLeader = new CANSparkMax(DrivetrainConstants.kLeftMotorLeaderCanId, MotorType.kBrushless);
  private final CANSparkMax m_rightLeader = new CANSparkMax(DrivetrainConstants.kRightMotorLeaderCanId, MotorType.kBrushless);
  private final CANSparkMax m_leftFollower = new CANSparkMax(DrivetrainConstants.kLeftMotorFollowerCanId, MotorType.kBrushless);
  private final CANSparkMax m_rightFollower = new CANSparkMax(DrivetrainConstants.kRightMotorFollowerCanId, MotorType.kBrushless);

  private final DoubleSolenoid m_shifters = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
    DrivetrainConstants.kShifterForwardsPcmId, DrivetrainConstants.kShifterReversePcmId);
  private Boolean isHighGear;

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
  	// Left side leader motor setup.
  	m_leftLeader.restoreFactoryDefaults();
  	
  	// Right side leader motor setup.
  	m_rightLeader.restoreFactoryDefaults();
  	m_rightLeader.setInverted(true);
  	
  	// Left side follower motor setup.
  	m_leftFollower.restoreFactoryDefaults();
  	m_leftFollower.follow(m_leftLeader);
  	
  	// Right side follower motor setup.
  	m_rightFollower.restoreFactoryDefaults();
  	m_rightFollower.follow(m_rightLeader);
  	
  	m_drive.setDeadband(0.1);

    // Since robot will be disabled when this initializes, it won't fire the solenoid.
    // (Does this fire as soon as it's enabled?)
    shiftGearboxHigh();
  }

  /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public CommandBase arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    return run(() -> m_drive.arcadeDrive(fwd.getAsDouble(), -rot.getAsDouble()))
        .withName("arcadeDrive");
  }

  public void shiftGearboxHigh() {
    m_shifters.set(Value.kForward);
    isHighGear = true;
  }

  public void shiftGearboxLow() {
    m_shifters.set(Value.kReverse);
    isHighGear = false;
  }

  public void toggleGearbox() {
    if(isHighGear == false) {
      shiftGearboxHigh();
    } else {
      shiftGearboxLow();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // If we want to do something fun here, we could maybe estimate the drivetrain's speed and report it to the dashboard?
  }
}
