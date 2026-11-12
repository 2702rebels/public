package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState.Intent;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import java.util.Optional;

public class AutoRoutines {
  private final AutoFactory factory;
  private final RobotState state;
  private final CommandSwerveDrivetrain drivetrain;
  private final Superstructure superstructure;

  public AutoRoutines(
      AutoFactory factory, CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
    this.factory = factory;
    this.drivetrain = drivetrain;
    this.superstructure = superstructure;
    this.state = RobotState.getInstance();
  }

  private int getLocationIndex(char c) {
    return c - 'A';
  }

  public AutoRoutine scoreL4x(char c) {
    final AutoRoutine routine = factory.newRoutine("Score L4 (" + c + ")");

    routine.active().onTrue(state.scoreAutoL4(getLocationIndex(c)));
    return routine;
  }

  private static final double kFieldWidth = FieldConstants.kFieldWidth.in(Meters);
  private static final double kStartIntakeY = 5.65; // meters
  private static final double kStopIntakeY = 5.75; // meters

  // spotless:off
  private static final Pose2d startIntakeLeft = new Pose2d(4, kStartIntakeY, Rotation2d.fromDegrees(-180));
  private static final Pose2d stopIntakeLeft = new Pose2d(0.85, kStopIntakeY, Rotation2d.fromDegrees(-180));
  private static final Pose2d startScoreLeft = new Pose2d(2.5, 6.3, Rotation2d.fromDegrees(310));

  private static final Pose2d startIntakeRight = new Pose2d(4, kFieldWidth - kStartIntakeY, Rotation2d.fromDegrees(-180));
  private static final Pose2d stopIntakeRight = new Pose2d(0.85, kFieldWidth - kStopIntakeY, Rotation2d.fromDegrees(-180));
  private static final Pose2d startScoreRight = new Pose2d(2.5, kFieldWidth - 6.3, Rotation2d.fromDegrees(50));
  // spotless:on

  /** Stops intake driving sequence once acquired coral or hit the field perimeter. */
  private boolean shouldStopIntakeSequence() {
    return superstructure.hasCoral.getAsBoolean()
        || !GameConstants.withinAutoFieldPerimeter(state.getRobotPose());
  }

  public AutoRoutine scoreL4IL() {
    return scoreL4xy('I', 'L', startIntakeLeft, stopIntakeLeft, startScoreLeft);
  }

  public AutoRoutine scoreL4FC() {
    return scoreL4xy('F', 'C', startIntakeRight, stopIntakeRight, startScoreRight);
  }

  public AutoRoutine scoreL4JL() {
    return scoreL4xy('J', 'L', startIntakeLeft, stopIntakeLeft, startScoreLeft);
  }

  public AutoRoutine scoreL4EC() {
    return scoreL4xy('E', 'C', startIntakeRight, stopIntakeRight, startScoreRight);
  }

  private AutoRoutine scoreL4xy(
      char target1, char target2, Pose2d startIntake, Pose2d stopIntake, Pose2d startScore2) {
    final AutoRoutine routine = factory.newRoutine("Score L4 (" + target1 + "," + target2 + ")");
    routine.active().onTrue(state.scoreAutoL4(getLocationIndex(target1)));

    final var intake =
        Commands.sequence(
            new DriveToPose(
                    drivetrain,
                    () -> Optional.of(FieldConstants.fieldPose.apply(startIntake)),
                    state::getRobotPose,
                    Constants.DriveConstants.maxSpeed.times(0.8),
                    Constants.DriveConstants.maxAcceleration.times(0.8))
                .withTimeout(1.1),
            state.setIntent(Intent.kCoralFloorIntake),
            new DriveToPose(
                    drivetrain,
                    () -> Optional.of(FieldConstants.fieldPose.apply(stopIntake)),
                    state::getRobotPose,
                    Constants.DriveConstants.maxSpeed.times(0.6),
                    Constants.DriveConstants.maxAcceleration.times(0.7))
                .until(this::shouldStopIntakeSequence));

    final var scoreTarget2 =
        Commands.sequence(
            new DriveToPose(
                    drivetrain, FieldConstants.fieldPose.apply(startScore2), state::getRobotPose)
                .withTimeout(2),
            state.scoreAutoL4(getLocationIndex(target2)));

    routine
        .active()
        .and(state.idle)
        .and(() -> state.getAutoScoringStep() == 1)
        .and(superstructure.hasCoral.negate())
        .onTrue(
            intake.andThen(
                Commands.either(
                    scoreTarget2,
                    Commands.runOnce(superstructure::disableIntake),
                    superstructure.hasCoral)));

    return routine;
  }

  public AutoRoutine naive(String name, Command command) {
    final AutoRoutine routine = factory.newRoutine(name);

    routine.active().onTrue(command);
    return routine;
  }
}
