package frc.robot.subsystems;

import com.revrobotics.spark.SparkLimitSwitch;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.Tunable;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Combines elevator and pivot since we want a coupled control of both during motion to the desired
 * position.
 */
public final class Superstructure extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kStowedHeight = 0;
  private static final double __kStowedAlgaeHeight = 20;
  private static final double __kFloorHeight = 0;
  private static final double __kL1Height = 16;
  private static final double __kL2Height = 34.5;
  private static final double __kL3Height = 50.5;
  private static final double __kL4Height = 64;
  private static final double __kL4SHeight = 46;
  private static final double __kL4RHeight = 63;
  private static final double __kL2AlgaeHeight = 32;
  private static final double __kL3AlgaeHeight = 48;
  private static final double __kEjectAlgaeHeight = 10;
  private static final double __kNetHeight = 67.5;
  private static final double __kHoverHeight = 12;

  // angles are in degrees [!]
  private static final double __kStowedAngle = 130;
  private static final double __kTransitionAngle = 75;
  private static final double __kStowedAlgaeAngle = 70;
  private static final double __kEjectAlgaeAngle = 0;
  private static final double __kTransitionAlgaeAngle = 70;
  private static final double __kFloorAngle = -1;
  private static final double __kL1Angle = 25;
  private static final double __kL2Angle = -6;
  private static final double __kL3Angle = -6;
  private static final double __kL4Angle = 80;
  private static final double __kL4SAngle = 80;
  private static final double __kL2AlgaeAngle = -5;
  private static final double __kL3AlgaeAngle = -5;
  private static final double __kNetAngle = 100;
  private static final double __kClimbAngle = 20;

  private static final double __kBranchOffset = 32;

  private static final double kSafeEjectAngle = 50;
  private static final double kAngleTolerance = 10; // for elevator protection
  private static final double kElevatorTolerance = 2;
  private static final double kElevatorToleranceNet = 4;
  private static final double kBranchDownwardDelta = 2;

  @Logged(name = "Position", importance = Importance.INFO)
  private Position position = Position.kStowed;

  // delay on the coral presence trigger to allow it fully
  // leave the mechanism before shutting down motors, etc.
  private static final double kHasCoralDelaySeconds = 0.2;

  // delay on the coral presence trigger (rising edge) to allow
  // motors to run longer when game piece is partially in
  private static final double kCoralIntakeDelaySeconds = 0.05;

  // delay on the elevator stall detection (rising edge)
  private static final double kElevatorStallDelaySeconds = 0.1;

  // debounce algae presence to distinguish between initial
  // amp surge and stall situations
  private static final double kAlgaeIntakeDelaySeconds = 0.25;

  // height when the pivot can reach beyond critical angle and elevator can move
  private static final double kElevatorCriticalHeight = 60;

  // profile captured when the position change was commenced
  private Optional<Double> heightAtLastPosition = Optional.empty();
  private Optional<Double> angleAtLastPosition = Optional.empty();

  private final Elevator elevator;
  private final Pivot pivot;

  private final SparkLimitSwitch coralSensor;

  // allows overriding elevator height
  private Optional<Double> overrideElevatorHeight = Optional.empty();

  // allows overriding pivot angle
  private Optional<Double> overridePivotAngle = Optional.empty();

  // allows delaying deployment of the superstructure during driving
  // to the auto scoring position
  private boolean delayed;

  // allows disabling intake (manipulator) without lifting the pivot;
  // used during autonomous when failed to intake the coral but lifting
  // pivot may lead to dangerous consequences, e.g. jamming algae
  private boolean intakeDisabled;

  public Superstructure(Elevator elevator, Pivot pivot, Manipulator manipulator) {
    this.elevator = elevator;
    this.pivot = pivot;

    coralSensor = manipulator.getCoralPresenceSensor();

    // spotless:off
    canEject = new Trigger(() -> pivot.isBelowAngle(kSafeEjectAngle));
    isStowed = new Trigger(() -> elevator.isAtPosition(kStowedHeight.get()) && pivot.isAtAngle(kStowedAngle.get()));
    canScoreL1 = new Trigger(() -> elevator.isAtPosition(kL1Height.get()) && pivot.isAtAngle(kL1Angle.get()));
    canScoreL2 = new Trigger(() -> elevator.isAtPosition(kL2Height.get()) && pivot.isAtAngle(kL2Angle.get()));
    canScoreL3 = new Trigger(() -> elevator.isAtPosition(kL3Height.get()) && pivot.isAtAngle(kL3Angle.get()));
    canScoreL4 = new Trigger(() -> elevator.isAtPosition(kL4Height.get()) && pivot.isAtAngle(kL4Angle.get()));
    canScoreAlgae = new Trigger(() -> elevator.isAtPosition(kNetHeight.get(), 1) && pivot.isAtAngle(kNetAngle.get(), 5));
    canEjectAlgae = new Trigger(() -> elevator.isAtPosition(kEjectAlgaeHeight.get(), 1) && pivot.isAtAngle(kEjectAlgaeAngle.get(), 5));
    // spotless:on

    // use higher tolerance on the intake, since the elevator/pivot is moved up
    // once the coral is consumed by the manipulator
    canFloorIntake =
        new Trigger(
            () ->
                elevator.isAtPosition(kFloorHeight.get(), 1)
                    && pivot.isAtAngle(kFloorAngle.get(), 10));

    canIntakeAlgae =
        new Trigger(
            () -> {
              switch (position) {
                case kL2Algae:
                  return elevator.isAtPosition(kL2AlgaeHeight.get())
                      && pivot.isAtAngle(kL2AlgaeAngle.get());
                case kL3Algae:
                  return elevator.isAtPosition(kL3AlgaeHeight.get())
                      && pivot.isAtAngle(kL3AlgaeAngle.get());
                case kGroundAlgae:
                  return elevator.isAtPosition(kFloorHeight.get(), 1)
                      && pivot.isAtAngle(kFloorAngle.get(), 10);
                default:
                  return false;
              }
            });

    // spotless:off
    hasCoral = new Trigger(coralSensor::isPressed).debounce(kHasCoralDelaySeconds, DebounceType.kFalling).debounce(kCoralIntakeDelaySeconds, DebounceType.kRising);
    hasAlgae = new Trigger(manipulator::hasAlgae).debounce(kAlgaeIntakeDelaySeconds, DebounceType.kBoth);
    isElevatorStalled = new Trigger(elevator::isStalled).debounce(kElevatorStallDelaySeconds, DebounceType.kRising);
    // spotless:on
  }

  /** Mechanism positions */
  public static enum Position {
    kStowed,
    kFloor,
    kL1,
    kL2,
    kL3,
    kL4,
    kL4Retry,
    kL4Scored,
    kL2Algae,
    kL3Algae,
    kGroundAlgae,
    kEjectAlgae,
    kNet,
    kHover, // special position to allow trapped game pieces to escape
    kClimb, // special position for climbing
    kReady, // special position for ready to score the coral
  }

  /** Trigger for coral presence. */
  @Logged(name = "HasCoral", importance = Importance.INFO)
  public final Trigger hasCoral;

  /** Trigger for algae presence. */
  @Logged(name = "HasAlgae", importance = Importance.INFO)
  public final Trigger hasAlgae;

  /** Trigger for pivot in stowed position. */
  @Logged(name = "IsStowed", importance = Importance.INFO)
  public final Trigger isStowed;

  /** Trigger for pivot position safe for ejection. */
  @Logged(name = "CanEject", importance = Importance.INFO)
  public final Trigger canEject;

  /** Trigger for superstructure ready for ejection of algae. */
  @Logged(name = "CanEjectAlgae", importance = Importance.INFO)
  public final Trigger canEjectAlgae;

  /** Trigger for superstructure ready for scoring of algae. */
  @Logged(name = "CanScoreAlgae", importance = Importance.INFO)
  public final Trigger canScoreAlgae;

  /** Trigger for superstructure ready for floor intake */
  @Logged(name = "CanFloorIntake", importance = Importance.INFO)
  public final Trigger canFloorIntake;

  /** Trigger for superstructure ready for scoring at L1 */
  @Logged(name = "CanScoreL1", importance = Importance.INFO)
  public final Trigger canScoreL1;

  /** Trigger for superstructure ready for scoring at L2 */
  @Logged(name = "CanScoreL2", importance = Importance.INFO)
  public final Trigger canScoreL2;

  /** Trigger for superstructure ready for scoring at L3 */
  @Logged(name = "CanScoreL3", importance = Importance.INFO)
  public final Trigger canScoreL3;

  /** Trigger for superstructure ready for scoring at L4 */
  @Logged(name = "CanScoreL4", importance = Importance.INFO)
  public final Trigger canScoreL4;

  /** Trigger for superstructure ready to intake algae (from the reef). */
  @Logged(name = "CanAlgaeIntake", importance = Importance.INFO)
  public final Trigger canIntakeAlgae;

  /** Trigger for stalled elevator. */
  @Logged(name = "IsElevatorStalled", importance = Importance.INFO)
  public final Trigger isElevatorStalled;

  /** Trigger for disabled intake (manipulator). */
  @Logged(name = "ShouldDisableIntake", importance = Importance.INFO)
  public final Trigger shouldDisableIntake = new Trigger(() -> intakeDisabled);

  /** Trigger for delayed deployment. */
  @Logged(name = "IsDelayed", importance = Importance.INFO)
  public final Trigger isDelayed = new Trigger(() -> delayed);

  public Command setPosition(Position position) {
    return runOnce(
            () -> {
              this.position = position;
              this.heightAtLastPosition = Optional.of(elevator.getPosition());
              this.angleAtLastPosition = Optional.of(pivot.getAngle());
            })
        .withName("SetPosition");
  }

  public Command setPosition(Supplier<Position> position) {
    return runOnce(
            () -> {
              this.position = position.get();
              this.heightAtLastPosition = Optional.of(elevator.getPosition());
              this.angleAtLastPosition = Optional.of(pivot.getAngle());
            })
        .withName("SetPosition");
  }

  public Command maintainPosition(Position position) {
    return run(() -> {
          this.position = position;
          this.heightAtLastPosition = Optional.empty();
          this.angleAtLastPosition = Optional.empty();
        })
        .withName("MaintainPosition");
  }

  public Command setHeightOverride(double value) {
    return runOnce(
            () -> {
              this.overrideElevatorHeight = Optional.of(value);
            })
        .withName("SetHeightOverride");
  }

  public Command resetHeightOverride() {
    return runOnce(
            () -> {
              this.overrideElevatorHeight = Optional.empty();
            })
        .withName("ResetHeightOverride");
  }

  public Command setAngleOverride(double value) {
    return runOnce(
            () -> {
              this.overridePivotAngle = Optional.of(value);
            })
        .withName("SetAngleOverride");
  }

  public Command resetAngleOverride() {
    return runOnce(
            () -> {
              this.overridePivotAngle = Optional.empty();
            })
        .withName("ResetAngleOverride");
  }

  public boolean isDoneScoringL4() {
    return elevator.isAtPosition(kL4SHeight.get());
  }

  /** Sets delayed flag on the superstructure to prevent deployment during auto driving. */
  public void setDelayed(boolean value) {
    delayed = value;
  }

  /** Disables intake (manipulator). */
  public void disableIntake() {
    intakeDisabled = true;
  }

  /** Enables intake (manipulator). */
  public void enableIntake() {
    intakeDisabled = false;
  }

  /** Returns effective transition angle that depends on type of game piece being controlled. */
  private double getTransitionAngle() {
    return hasAlgae.getAsBoolean() ? kTransitionAlgaeAngle.get() : kTransitionAngle.get();
  }

  @Override
  public void periodic() {
    // reset elevator if we read negative position values
    if (elevator.getPosition() < 0) {
      elevator.reset();
    }

    // allows testing superstructure by manually commanding position and angle
    if (kManualEnabled.get()) {
      pivot.setAngle(kManualAngle.get());
      elevator.setPosition(kManualHeight.get());
      return;
    }

    final var setpoint = getSetpoint();
    final var pivotSetpoint = setpoint.getSecond();

    // safety: do not move elevator if the pivot is beyond critical (transition) angle
    // except for when above 60" (for net scoring)
    final var pivotAngle = pivot.getAngle();
    final var elevatorSetpoint =
        pivotAngle <= kTransitionAngle.get() + kAngleTolerance
                || elevator.getPosition() >= kElevatorCriticalHeight
            ? setpoint.getFirst()
            : elevator.getPosition();

    pivot.setAngle(overridePivotAngle.isPresent() ? overridePivotAngle.get() : pivotSetpoint);
    elevator.setPosition(
        overrideElevatorHeight.isPresent() ? overrideElevatorHeight.get() : elevatorSetpoint);
  }

  /** Returns the next setpoint for elevator and pivot. */
  private Pair<Double, Double> getSetpoint() {
    final var height = elevator.getPosition();
    final var profile = positionProfiles.get(position);

    // override stowed angle/height if we think we have an algae
    final var targetHeight =
        position == Position.kStowed && hasAlgae.getAsBoolean()
            ? kStowedAlgaeHeight.get()
            : profile.getFirst();

    final var targetAngle =
        position == Position.kStowed && hasAlgae.getAsBoolean()
            ? kStowedAlgaeAngle.get()
            : profile.getSecond();

    // we should only transition between different "L4" levels by maintaining angle,
    // no additional logic is needed, just command new height (and angle)
    if (position == Position.kL4Scored || position == Position.kL4Retry) {
      return Pair.of(targetHeight, targetAngle);
    }

    final var transitionAngle = getTransitionAngle();

    // keep pivot at transition angle outside the zone centered at the target height
    // safety: do not allow pivot to go beyond transition angle except at stow position,
    // near zero, or at net position, near top
    final var pivotProtected =
        height > kElevatorTolerance && height < kNetHeight.get() - kElevatorToleranceNet;

    var actualAngle =
        MathUtil.isNear(targetHeight, height, kBranchOffset.get())
            ? pivotProtected ? Math.min(targetAngle, transitionAngle) : targetAngle
            : transitionAngle;

    // on downward motion do not stow the pivot right away since it tends to hit
    // the coral above it as it retracts, instead wait until elevator lowers somewhat
    if (targetHeight < height - kElevatorTolerance
        && heightAtLastPosition.isPresent()
        && angleAtLastPosition.isPresent()) {
      final var _heightAtLastPosition = heightAtLastPosition.get();
      if (_heightAtLastPosition > kL1Height.get()
          && height > _heightAtLastPosition - kBranchDownwardDelta) {

        // always force it at least to transition angle if we are too far out
        // e.g. when scoring in the net
        actualAngle = Math.min(angleAtLastPosition.get(), transitionAngle);
      }
    }

    return Pair.of(targetHeight, actualAngle);
  }

  private static Map<Position, Pair<Double, Double>> positionProfiles = new HashMap<>();

  private static void updatePositionProfiles(double _value) {
    // we prefer to cache profiles for each position since these do not change once tuned,
    // but we are constantly fetching them in the periodic to determine where mechanism must go,
    // caching prevents unnecessary allocations and improves lookup time

    // spotless:off
    positionProfiles.put(Position.kStowed, Pair.of(kStowedHeight.get(), kStowedAngle.get()));
    positionProfiles.put(Position.kFloor, Pair.of(kFloorHeight.get(), kFloorAngle.get()));
    positionProfiles.put(Position.kL1, Pair.of(kL1Height.get(), kL1Angle.get()));
    positionProfiles.put(Position.kL2, Pair.of(kL2Height.get(), kL2Angle.get()));
    positionProfiles.put(Position.kL3, Pair.of(kL3Height.get(), kL3Angle.get()));
    positionProfiles.put(Position.kL4, Pair.of(kL4Height.get(), kL4Angle.get()));
    positionProfiles.put(Position.kL4Scored, Pair.of(kL4SHeight.get(), kL4SAngle.get()));
    positionProfiles.put(Position.kL4Retry, Pair.of(kL4RHeight.get(), kL4SAngle.get()));
    positionProfiles.put(Position.kL2Algae, Pair.of(kL2AlgaeHeight.get(), kL2AlgaeAngle.get()));
    positionProfiles.put(Position.kL3Algae, Pair.of(kL3AlgaeHeight.get(), kL3AlgaeAngle.get()));
    positionProfiles.put(Position.kGroundAlgae, Pair.of(kFloorHeight.get(), kFloorAngle.get()));
    positionProfiles.put(Position.kNet, Pair.of(kNetHeight.get(), kNetAngle.get()));
    positionProfiles.put(Position.kEjectAlgae, Pair.of(kEjectAlgaeHeight.get(), kEjectAlgaeAngle.get()));
    positionProfiles.put(Position.kHover, Pair.of(kHoverHeight.get(), kTransitionAngle.get()));
    positionProfiles.put(Position.kClimb, Pair.of(kFloorHeight.get(), kClimbAngle.get()));
    positionProfiles.put(Position.kReady, Pair.of(kStowedHeight.get(), kTransitionAngle.get()));
    // spotless:on
  }

  private static Tunable.DoubleValue profileValue(String name, double defaultValue) {
    return Tunable.value(
        "Superstructure/" + name, defaultValue, Superstructure::updatePositionProfiles);
  }

  private static Tunable.BooleanValue profileValue(String name, boolean defaultValue) {
    return Tunable.value("Superstructure/" + name, defaultValue);
  }

  // spotless:off
  private static Tunable.DoubleValue kStowedAngle = profileValue("Stowed/Angle", __kStowedAngle);
  private static Tunable.DoubleValue kStowedHeight = profileValue("Stowed/Height", __kStowedHeight);
  private static Tunable.DoubleValue kFloorAngle = profileValue("Floor/Angle", __kFloorAngle);
  private static Tunable.DoubleValue kFloorHeight = profileValue("Floor/Height", __kFloorHeight);
  private static Tunable.DoubleValue kL1Angle = profileValue("L1/Angle", __kL1Angle);
  private static Tunable.DoubleValue kL1Height = profileValue("L1/Height", __kL1Height);
  private static Tunable.DoubleValue kL2Angle = profileValue("L2/Angle", __kL2Angle);
  private static Tunable.DoubleValue kL2Height = profileValue("L2/Height", __kL2Height);
  private static Tunable.DoubleValue kL3Angle = profileValue("L3/Angle", __kL3Angle);
  private static Tunable.DoubleValue kL3Height = profileValue("L3/Height", __kL3Height);
  private static Tunable.DoubleValue kL4Angle = profileValue("L4/Angle", __kL4Angle);
  private static Tunable.DoubleValue kL4Height = profileValue("L4/Height", __kL4Height);
  private static Tunable.DoubleValue kL4SAngle = profileValue("L4Scored/Angle", __kL4SAngle);
  private static Tunable.DoubleValue kL4SHeight = profileValue("L4Scored/Height", __kL4SHeight);
  private static Tunable.DoubleValue kL4RHeight = profileValue("L4Retry/Height", __kL4RHeight);
  private static Tunable.DoubleValue kL2AlgaeAngle = profileValue("L2Algae/Angle", __kL2AlgaeAngle);
  private static Tunable.DoubleValue kL2AlgaeHeight = profileValue("L2Algae/Height", __kL2AlgaeHeight);
  private static Tunable.DoubleValue kL3AlgaeAngle = profileValue("L3Algae/Angle", __kL3AlgaeAngle);
  private static Tunable.DoubleValue kL3AlgaeHeight = profileValue("L3Algae/Height", __kL3AlgaeHeight);
  private static Tunable.DoubleValue kNetHeight = profileValue("Net/Height", __kNetHeight);
  private static Tunable.DoubleValue kNetAngle = profileValue("Net/Angle", __kNetAngle);
  private static Tunable.DoubleValue kHoverHeight = profileValue("HoverHeight", __kHoverHeight);
  private static Tunable.DoubleValue kTransitionAngle = profileValue("TransitionAngle", __kTransitionAngle);
  private static Tunable.DoubleValue kTransitionAlgaeAngle = profileValue("TransitionAlgaeAngle", __kTransitionAlgaeAngle);
  private static Tunable.DoubleValue kStowedAlgaeAngle = profileValue("StowedAlgaeAngle", __kStowedAlgaeAngle);
  private static Tunable.DoubleValue kStowedAlgaeHeight = profileValue("StowedAlgaeHeight", __kStowedAlgaeHeight);
  private static Tunable.DoubleValue kEjectAlgaeAngle = profileValue("EjectAlgaeAngle", __kEjectAlgaeAngle);
  private static Tunable.DoubleValue kEjectAlgaeHeight = profileValue("EjectAlgaeHeight", __kEjectAlgaeHeight);
  private static Tunable.DoubleValue kClimbAngle = profileValue("ClimbAngle", __kClimbAngle);
  private static Tunable.DoubleValue kBranchOffset = profileValue("BranchOffset", __kBranchOffset);

  private static Tunable.DoubleValue kManualAngle = profileValue("Manual/Angle", 0);
  private static Tunable.DoubleValue kManualHeight = profileValue("Manual/Height", 0);
  private static Tunable.BooleanValue kManualEnabled = profileValue("Manual/Enabled", false);
  // spotless:on

  static {
    updatePositionProfiles(0); // value is ignored
  }
}
