package frc.robot.subsystems.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Misc.ColorHSV;
import java.util.function.BiConsumer;

public class Led extends SubsystemBase {
  private final AddressableLED strip;
  private final AddressableLEDBuffer buffer;

  private Color color;
  private BiConsumer<AddressableLEDBuffer, Double> pattern;

  private double period = 0;
  private double periodStart;
  private double periodLast;
  private int periodDiscretization;
  private boolean isTracking;
  private boolean isTrackingBoth;

  /** Pattern that does nothing */
  private static void nop(AddressableLEDBuffer buffer, double t) {}

  public Led(int length, int port) {
    // initialize the physical LED strip length
    strip = new AddressableLED(port);
    strip.setLength(length);

    // black color means LEDs are in the off state
    color = Color.kBlack;
    buffer = new AddressableLEDBuffer(length);

    pattern = Led::nop;
    periodStart = Timer.getFPGATimestamp();

    // start the physical LED strip operation
    strip.start();
  }

  /**
   * Sets the solid color to use with standard patterns.
   *
   * <p>You can set this without changing the pattern to switch to a different color.
   */
  public void setColor(Color color) {
    this.color = color;
  }

  /**
   * Sets the period in seconds and optional discretization.
   *
   * <p>You can set this without changing the pattern to switch to a different period.
   *
   * @param period period in seconds
   * @param samples number of samples for period discretization or 0 for continuous updates
   */
  public void setPeriod(double period, int samples) {
    this.period = period;
    this.periodStart = Timer.getFPGATimestamp();
    this.periodDiscretization = samples;
  }

  /**
   * Sets the period in seconds with continuous updates.
   *
   * <p>You can set this without changing the pattern to switch to a different period.
   *
   * @param period period in seconds
   */
  public void setPeriod(double period) {
    this.setPeriod(period, 0);
  }

  /**
   * Sets the pattern to the specified functional implementation, period and discretization.
   *
   * <p>Setting the pattern this way allows for arbitrary implementations and will disregard current
   * color settings.
   */
  public void setPattern(
      BiConsumer<AddressableLEDBuffer, Double> pattern, double period, int samples) {
    this.pattern = pattern;
    setPeriod(period, samples);
  }

  /**
   * Sets the pattern to the specified functional implementation and period.
   *
   * <p>Setting the pattern this way allows for arbitrary implementations and will disregard current
   * color settings.
   */
  public void setPattern(BiConsumer<AddressableLEDBuffer, Double> pattern, double period) {
    this.setPattern(pattern, period, 0);
  }

  @Override
  public void periodic() {
    final var timestamp = Timer.getFPGATimestamp();
    final var t =
        MathUtil.clamp(period > 0 ? ((timestamp - periodStart) % period) / period : 0, 0, 1);

    // do not fire updates until next discretization sample
    if (periodDiscretization > 0 && (timestamp - periodLast) < (period / periodDiscretization)) {
      return;
    }

    // populate the buffer using pattern implementation
    // and write the buffer to the physical LED strip
    pattern.accept(buffer, Double.valueOf(t));

    // override the buffer portion to convey vision information
    Color trackingColour;
    if (isTracking) {
      trackingColour = isTrackingBoth ? Color.kPurple : Color.kGreen;
    } else {
      trackingColour = Color.kRed;
    }

    for (int i = LedConstants.VISION[0]; i <= LedConstants.VISION[1]; ++i) {
      buffer.setLED(i, trackingColour);
    }
    for (int i = LedConstants.VISION_SHOOTER_SIDE[0];
        i <= LedConstants.VISION_SHOOTER_SIDE[1];
        i++) {
      buffer.setLED(i, trackingColour);
    }

    strip.setData(buffer);

    periodLast = timestamp;
  }

  /** Sets the solid color pattern. */
  public void solid(Color color, double period) {
    // initialize buffer here (once) and use 'nop' pattern,
    // since we don't need to update the buffer periodically
    // when solid color is requested
    LedPatterns.setColor(buffer, color);
    setColor(color);
    setPattern(Led::nop, period);
  }

  /** Sets the flashing color pattern. */
  public void flash(Color color, double period) {
    setColor(color);
    setPattern((buffer, t) -> LedPatterns.flash(buffer, t, this.color), period);
  }

  /** Sets the countdown color pattern. */
  public void countdown(Color color, double period, int chunks) {
    setColor(color);
    setPattern((buffer, t) -> LedPatterns.countdown(buffer, t, this.color, chunks), period);
  }

  /** Sets the countup color pattern. */
  public void countup(Color color, double period, int chunks) {
    setColor(color);
    setPattern((buffer, t) -> LedPatterns.countup(buffer, t, this.color, chunks), period);
  }

  /** Sets the breath color pattern. */
  public void breath(Color color, double period) {
    setColor(color);
    setPattern((buffer, t) -> LedPatterns.breath(buffer, t, this.color, 0.1, 1), period);
  }

  /** Sets the breath color pattern using HSV color scheme. */
  public void breath(ColorHSV color, double period) {
    setPattern((buffer, t) -> LedPatterns.breath(buffer, t, color, 0.1, 1), period);
  }

  /** Sets the random color pattern using HSV color scheme. */
  public void random(ColorHSV color, double period, int samples) {
    setPattern((buffer, t) -> LedPatterns.random(buffer, t, color), period, samples);
  }

  /** Sets indicator of the visual target tracking. */
  public void setTracking(boolean isTracking) {
    this.isTracking = isTracking;
  }

  public void setTrackingBoth(boolean secondTags) {
    this.isTrackingBoth = this.isTracking && secondTags;
  }
}
