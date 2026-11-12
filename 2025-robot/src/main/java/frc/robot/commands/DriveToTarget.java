package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;
import java.util.function.Supplier;

public class DriveToTarget extends Command {
  private final Supplier<Optional<Pose2d>> targetPoseSupplier;
  private final Supplier<LinearVelocity> velocitySupplier;
  private final CommandSwerveDrivetrain drive;
  private Optional<Pose2d> lastTarget = Optional.empty();
  private boolean running;

  public DriveToTarget(
      CommandSwerveDrivetrain drive,
      Supplier<Optional<Pose2d>> targetPoseSupplier,
      Supplier<LinearVelocity> velocitySupplier) {
    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
    this.velocitySupplier = velocitySupplier;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    lastTarget = targetPoseSupplier.get();
  }

  @Override
  public void execute() {
    final var maybeTarget = targetPoseSupplier.get();
    if (maybeTarget.isPresent()) {
      lastTarget = maybeTarget;
    }

    if (lastTarget.isEmpty()) {
      return;
    }

    running = true;
    final var targetPose = lastTarget.get();

    // use orientation lock onto target
    drive.setOrientationLock(targetPose.getRotation());
    drive.driveAtRobotRelative(velocitySupplier.get().in(MetersPerSecond), 0, 0);
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    drive.clearOrientationLock();
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return !lastTarget.isPresent();
  }

  public boolean isRunning() {
    return running;
  }
}
