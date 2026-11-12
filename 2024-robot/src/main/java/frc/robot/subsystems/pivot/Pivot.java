package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  // spotless:off
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/Motor P", 1.0);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/Motor I", 0.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/Motor D", 0.0);

  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/ArmFF kS", 0.01);
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Pivot/ArmFF kG", 0.44);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/ArmFF kV", 2.57);
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Pivot/ArmFF kA", 0.02);

  private static final LoggedTunableNumber maxVelocity = new LoggedTunableNumber("Pivot/Profile/Max Velocity", 0.0);
  private static final LoggedTunableNumber maxAcceleration = new LoggedTunableNumber("Pivot/Profile/Max Acceleration", 0.0);

  private static final LoggedTunableNumber minPivotAngle = new LoggedTunableNumber("Pivot/Limit/Min Angle", 15.0);
  private static final LoggedTunableNumber maxPivotAngle = new LoggedTunableNumber("Pivot/Limit/Max Angle", 110.0);
  private static final LoggedTunableNumber onGoalAllowance = new LoggedTunableNumber("Pivot/OnGoalAllowance", 1.0);
  // spotless:on

  private ArmFeedforward armFeedforward;
  private TrapezoidProfile trapezoidProfile;

  private TrapezoidProfile.State state = new State(0, 0);
  private TrapezoidProfile.State goal = new State(0, 0);

  private boolean enabled = false;

  private double requestedPosInDeg = 0;

  public Pivot(PivotIO io) {
    this.io = io;
    this.io.configureJointPID(kP.get(), kI.get(), kD.get());

    armFeedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
    trapezoidProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot/Inputs", inputs);

    final var callerId = hashCode();
    if (kP.hasChanged(callerId) || kI.hasChanged(callerId) || kD.hasChanged(callerId)) {
      System.out.println("[Pivot] Joint PID parameters have changed. Reconfiguring them.");
      io.configureJointPID(kP.get(), kI.get(), kD.get());
    }

    if (kS.hasChanged(callerId)
        || kG.hasChanged(callerId)
        || kV.hasChanged(callerId)
        || kA.hasChanged(callerId)) {
      System.out.println("[Pivot] ArmFeedForward parameters have changed. Reconfiguring them.");
      armFeedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
    }

    if (maxVelocity.hasChanged(callerId) || maxAcceleration.hasChanged(callerId)) {
      System.out.println("[Pivot] TrapezoidProfile parameters have changed. Reconfiguring them.");
      trapezoidProfile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    }

    goal.position = Math.toRadians(requestedPosInDeg);
    state.position = inputs.measuredPositionRadians;
    state.velocity = inputs.measuredVelocityRadiansPerSecond;

    // this calculate method is doing at least two new allocations... :(
    state = trapezoidProfile.calculate(Constants.cyclePeriodSeconds, state, goal);
    final var ff = armFeedforward.calculate(state.position, state.velocity);

    Logger.recordOutput("Pivot/RequestedPosition", requestedPosInDeg);
    Logger.recordOutput("Pivot/StatePosition", state.position);
    Logger.recordOutput("Pivot/StateVelocity", state.velocity);
    Logger.recordOutput("Pivot/ArmFeedForward", ff);
    Logger.recordOutput("Pivot/PositionRadians", inputs.measuredPositionRadians);
    Logger.recordOutput("Pivot/PositionAngle", Math.toDegrees(inputs.measuredPositionRadians));

    if (enabled) {
      io.setJointAngle(state.position, ff);
    } else {
      io.stop();
    }
  }

  /** Disable pivot's control loop. */
  public void disable() {
    enabled = false;
    System.out.println("[Pivot] Disabling pivot control loop.");
  }

  /** Sets the requested angle for the pivot's control loop. */
  public void setRequestedAngle(double angle) {
    if (!enabled) {
      enabled = true;
      System.out.println("[Pivot] Enabling pivot control loop.");
    }

    requestedPosInDeg = MathUtil.clamp(angle, minPivotAngle.get(), maxPivotAngle.get());
  }

  /** Returns the pivot's current angle in degrees. Zero degrees is horizontal with the ground. */
  public double getCurrentAngle() {
    return Math.toDegrees(inputs.measuredPositionRadians);
  }

  /** Checks if the pivot is at it's requested goal. With + or - the allowance. */
  public boolean isAtGoal(double allowance) {
    final double angle = getCurrentAngle();
    return angle < requestedPosInDeg + allowance && angle > requestedPosInDeg - allowance;
  }

  /** Checks if the pivot is at it's requested goal. Uses default onGoalAllowance parameters. */
  public boolean isAtGoal() {
    return isAtGoal(onGoalAllowance.get());
  }
}
