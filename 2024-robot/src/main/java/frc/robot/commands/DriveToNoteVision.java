package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.NoteDetector;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class DriveToNoteVision extends Command {
  private static final LoggedTunableNumber thetaTol =
      new LoggedTunableNumber("Drive/Note/Theta Tolerance", 2);
  private static final LoggedTunableNumber velocityLinearTeleop =
      new LoggedTunableNumber("Drive/Note/Velocity Teleop", 3);
  private static final LoggedTunableNumber velocityLinearSlow =
      new LoggedTunableNumber("Drive/Note/Velocity Slow", 0.75);
  private static final LoggedTunableNumber velocityLinearFast =
      new LoggedTunableNumber("Drive/Note/Velocity Fast", 1.5);

  private final Drive drive;
  private final NoteDetector detector;
  private final Intake intake;
  private final Shooter shooter;
  private boolean targetAcquired;
  private boolean directionLocked;
  private double desiredTheta;

  public DriveToNoteVision(Drive drive, NoteDetector detector, Intake intake, Shooter shooter) {
    this.drive = drive;
    this.detector = detector;
    this.intake = intake;
    this.shooter = shooter;

    addRequirements(drive, detector);
  }

  @Override
  public void initialize() {
    final var note = detector.getTrackedNote();
    directionLocked = false;

    if (note == null) {
      targetAcquired = false;
    } else {
      targetAcquired = true;
      desiredTheta = drive.getPose().getRotation().getRadians() - note.yaw().getRadians();
    }

    System.out.println("DriveToNoteVision/TargetAcquired = " + targetAcquired);
  }

  @Override
  public void execute() {
    if (!targetAcquired) {
      return;
    }

    final var currentTheta = drive.getPose().getRotation().getRadians();
    final var turnAngle = currentTheta - desiredTheta;
    drive.steeringLock(turnAngle);

    Logger.recordOutput("DriveToNoteVision/TurnAngle", turnAngle);

    // once the direction is locked disregard further theta readings
    if (Math.abs(turnAngle) < Math.toRadians(thetaTol.get())) {
      directionLocked = true;
    }

    final var speedFast =
        DriverStation.isTeleopEnabled() ? velocityLinearTeleop.get() : velocityLinearFast.get();

    final var vx =
        directionLocked && !shooter.isFrontBeamTripped()
            ? intake.isBeamTripped() ? velocityLinearSlow.get() : speedFast
            : 0;

    // steering is handled by the lock
    drive.runVelocity(new ChassisSpeeds(vx, 0, 0));
  }

  @Override
  public void end(boolean interrupted) {
    drive.steeringUnlock();
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return !targetAcquired || shooter.isFrontBeamTrippedAndTimeout();
  }
}
