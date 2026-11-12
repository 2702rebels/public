package frc.robot.subsystems;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DefaultCommand;
import java.util.Map;
import java.util.function.Supplier;

public class LedVision extends SubsystemBase {
  public static enum Type {
    kNone,

    /** Unknown/non-trusted odometry */
    kUnknown,

    /** Trusted odometry */
    kTrusted,
  }

  private final LedResource resource;
  private final Supplier<Type> supplier;

  public LedVision(LedResource resource, Supplier<Type> supplier) {
    this.resource = resource;
    this.supplier = supplier;

    // set default command
    setDefaultCommand(createDefault());
  }

  // "evergreen" patterns
  private static LEDPattern assistPattern = LedResource.blink(Color.kGreen);

  // specific patterns
  private static final Map<Type, LEDPattern> kStatePatterns =
      Map.of(
          Type.kNone, LEDPattern.kOff,
          Type.kUnknown, LEDPattern.solid(Color.kDarkRed),
          Type.kTrusted, LEDPattern.solid(Color.kGreen));

  private Command runDefault(Runnable toRun) {
    return new DefaultCommand(toRun, this);
  }

  /** Creates default command for this LED subsystem that is state-driven */
  private Command createDefault() {
    return run(() -> {
          resource.setVisionPattern(kStatePatterns.get(supplier.get()));
        })
        .ignoringDisable(true)
        .withName("LedVisionDefault");
  }

  /** Commands LED strip to indicate vision-assisted driving / sequence. */
  public Command signalAssist() {
    return runDefault(() -> resource.setVisionPattern(assistPattern));
  }
}
