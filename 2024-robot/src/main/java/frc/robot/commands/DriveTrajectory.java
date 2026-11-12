package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class DriveTrajectory extends SequentialCommandGroup {
  private static final LoggedTunableNumber driveP =
      new LoggedTunableNumber("Drive/Trajectory/Drive P", 0.1);
  private static final LoggedTunableNumber driveI =
      new LoggedTunableNumber("Drive/Trajectory/Drive I", 0.0);
  private static final LoggedTunableNumber driveD =
      new LoggedTunableNumber("Drive/Trajectory/Drive D", 0.0);
  private static final LoggedTunableNumber thetaP =
      new LoggedTunableNumber("Drive/Trajectory/Theta P", 0.1);
  private static final LoggedTunableNumber thetaI =
      new LoggedTunableNumber("Drive/Trajectory/Theta I", 0.0);
  private static final LoggedTunableNumber thetaD =
      new LoggedTunableNumber("Drive/Trajectory/Theta D", 0.0);

  public DriveTrajectory(
      ChoreoTrajectory trajectory, boolean resetOdometry, boolean stopAtEnd, Drive drive) {

    final var driveControllerX = new PIDController(driveP.get(), driveI.get(), driveD.get());
    final var driveControllerY = new PIDController(driveP.get(), driveI.get(), driveD.get());
    final var thetaController = new PIDController(thetaP.get(), thetaI.get(), thetaD.get());
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    final var swerveCommand =
        Choreo.choreoSwerveCommand(
            trajectory,
            drive::getPose,
            driveControllerX,
            driveControllerY,
            thetaController,
            (ChassisSpeeds speeds) -> drive.runVelocity(speeds),
            () ->
                DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red,
            drive);

    if (resetOdometry)
      addCommands(Commands.runOnce(() -> drive.setPose(trajectory.getInitialPose())));

    addCommands(swerveCommand);

    if (stopAtEnd) addCommands(Commands.runOnce(() -> drive.stop()));
  }
}
