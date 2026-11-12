package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DefaultCommand;
import java.util.Map;
import java.util.function.Supplier;

public class LedFeedback extends SubsystemBase {
  public static enum Type {
    kNone,

    /** Scoring coral sequence active */
    kScoringCoral,

    /** Scoring algae sequence active */
    kScoringAlgae,

    /** Intaking coral sequence active */
    kIntakingCoral,

    /** Intaking algae sequence active */
    kIntakingAlgae,

    /** Holding coral and taxing */
    kHoldingCoral,

    /** Holding algae and taxing */
    kHoldingAlgae,

    /** Climbing sequence active */
    kClimbing,
  }

  private final LedResource resource;
  private final Supplier<Type> supplier;

  public LedFeedback(LedResource resource, Supplier<Type> supplier) {
    this.resource = resource;
    this.supplier = supplier;

    // set default command
    setDefaultCommand(createDefault());
  }

  // "evergreen" patterns
  private static LEDPattern defaultPatternRed = LEDPattern.solid(Color.kRed);
  private static LEDPattern defaultPatternBlue = LEDPattern.solid(Color.kBlue);
  private static LEDPattern disabledPattern = LedResource.breathe(Color.kPurple, 5);
  private static LEDPattern estoppedPattern = LedResource.blink(Color.kPurple);
  private static LEDPattern successPattern = LedResource.blink(Color.kYellow);
  private static LEDPattern readyPattern = LedResource.blink(Color.kWhite);
  private static LEDPattern attainmentPattern = LEDPattern.solid(Color.kGreen);

  // game-specific patterns
  private static final Map<Type, LEDPattern> kStatePatterns =
      Map.ofEntries(
          Map.entry(Type.kScoringCoral, LedResource.breathe(Color.kWhite, 3)),
          Map.entry(Type.kScoringAlgae, LedResource.breathe(Color.kSpringGreen, 3)),
          Map.entry(Type.kIntakingCoral, LedResource.sweep(Color.kWhite)),
          Map.entry(Type.kIntakingAlgae, LedResource.sweep(Color.kSpringGreen)),
          Map.entry(Type.kHoldingCoral, LEDPattern.solid(Color.kWhite)),
          Map.entry(Type.kHoldingAlgae, LEDPattern.solid(Color.kSpringGreen)),
          Map.entry(Type.kClimbing, LedResource.wave(Color.kYellow)));

  private Command runDefault(Runnable toRun) {
    return new DefaultCommand(toRun, this);
  }

  /** Creates default command for this LED subsystem that is state-driven */
  private Command createDefault() {
    return run(() -> {
          var pattern = LEDPattern.kOff;

          // special handling of estopped/disabled/default states
          if (DriverStation.isEStopped()) {
            pattern = estoppedPattern;
          } else if (DriverStation.isDisabled()) {
            pattern = disabledPattern;
          } else {
            final var state = supplier.get(); // poll current state
            if (state == Type.kNone) {
              // select pattern based on the alliance color
              final var alliance = DriverStation.getAlliance();
              pattern =
                  alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red
                      ? defaultPatternRed
                      : defaultPatternBlue;
            } else {
              pattern = kStatePatterns.get(state);
            }
          }

          resource.setFeedbackPattern(pattern);
        })
        .ignoringDisable(true)
        .withName("LedFeedbackDefault");
  }

  /** Commands LED strip to indicate an action completed successfully. */
  public Command signalSuccess() {
    return runDefault(() -> resource.setFeedbackPattern(successPattern)).withTimeout(0.8);
  }

  /** Commands LED strip to indicate a ready state. */
  public Command signalReady() {
    return runDefault(() -> resource.setFeedbackPattern(readyPattern));
  }

  /** Commands LED strip to indicate a position attained state. */
  public Command signalAttainment() {
    return runDefault(() -> resource.setFeedbackPattern(attainmentPattern));
  }
}
