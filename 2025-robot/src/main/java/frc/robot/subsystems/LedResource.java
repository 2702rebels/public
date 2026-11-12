package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Map;

/**
 * Owns actual hardware (addressable LED) but is not used directly for scheduling, instead one
 * should use {@link LedFeedback} and @{link LedVision} subsystems for commanding feedback based on
 * the robot game and vision states.
 */
public class LedResource extends SubsystemBase {
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;

  private final AddressableLEDBufferView[] segments =
      new AddressableLEDBufferView[Constants.LedConstants.kSegments.length];

  private LEDPattern patternFeedback = LEDPattern.kOff;
  private LEDPattern patternVision = LEDPattern.kOff;

  /** Determines if the segment represents a tag tracking one. */
  private boolean isVisionSegment(int index) {
    return index == 0 || index == (segments.length / 2) || index == segments.length - 1;
  }

  public LedResource(int port) {
    leds = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(Constants.LedConstants.kLength);
    leds.setLength(buffer.getLength());

    // LED strip is arranged into segments as follows,
    // indexed from the right bottom:
    //
    // left     tracking   right
    // |           |           |
    // ****** ... *** ... ******
    // * -    right/left     - *
    // *                       *
    // .                       .
    // *                       *
    // * -     tracking      - *
    // * -                   - 0

    var offset = 0;
    for (var i = 0; i < segments.length; ++i) {
      final var length = Constants.LedConstants.kSegments[i];
      final var segment = buffer.createView(offset, offset + length - 1);

      segments[i] = i >= segments.length / 2 ? segment.reversed() : segment;
      offset += length;
    }

    leds.setData(buffer);
    leds.start();
  }

  public void setFeedbackPattern(LEDPattern pattern) {
    patternFeedback = pattern == null ? LEDPattern.kOff : pattern;
  }

  public void setVisionPattern(LEDPattern pattern) {
    patternVision = pattern == null ? LEDPattern.kOff : pattern;
  }

  @Override
  public void setDefaultCommand(Command defaultCommand) {
    throw new AssertionError("This subsystem should not be used directly");
  }

  @Override
  public void periodic() {
    for (var i = 0; i < segments.length; ++i) {
      if (isVisionSegment(i)) {
        patternVision.applyTo(segments[i]);
      } else {
        patternFeedback.applyTo(segments[i]);
      }
    }

    leds.setData(buffer);
  }

  // mask used with a sweep pattern
  private static final LEDPattern kMask =
      LEDPattern.steps(Map.of(0.0, Color.kWhite, 0.2, Color.kBlack))
          .scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(350));

  /** Creates a "breathe" pattern of the specified color. */
  public static LEDPattern breathe(Color color, double time) {
    return LEDPattern.solid(color).breathe(Units.Seconds.of(time));
  }

  /** Creates a "fast blink" pattern of the specified color. */
  public static LEDPattern blink(Color color) {
    return LEDPattern.solid(color).blink(Units.Second.of(0.15));
  }

  /** Creates a "sweep" pattern of the specified color. */
  public static LEDPattern sweep(Color color) {
    return LEDPattern.solid(color).mask(kMask);
  }

  /** Creates a "wave" pattern of the specified color. */
  public static LEDPattern wave(Color color) {
    return LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBlack, color)
        .scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(100))
        .reversed();
  }
}
