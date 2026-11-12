package frc.robot.subsystems.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.Misc.ColorHSV;
import java.util.Random;

/**
 * LED pattern implementations.
 *
 * <p>Each function is expected to have at least the following parameters: a buffer to be populated,
 * a parameter [0-1] indicating the progress within the current time period.
 *
 * <p>Some implementations may require additional parameters that are specific to the behavior.
 */
public final class LedPatterns {
  /** Sets the buffer to all zeros. */
  public static void setZero(AddressableLEDBuffer buffer) {
    for (var i = 0; i < buffer.getLength(); ++i) {
      buffer.setRGB(i, 0, 0, 0);
    }
  }

  /** Sets the buffer to the solid color. */
  public static void setColor(AddressableLEDBuffer buffer, Color color) {
    for (var i = 0; i < buffer.getLength(); ++i) {
      buffer.setLED(i, color);
    }
  }

  /** Implements flash LED pattern with the solid color. */
  public static void flash(AddressableLEDBuffer buffer, double t, Color color) {
    if (t >= 0.5) {
      setColor(buffer, color);
    } else {
      setZero(buffer);
    }
  }

  /**
   * Implements countdown LED pattern with the solid color.
   *
   * <p>The implementation assumes that LED strip is split into a set of equally sized chunks, that
   * should each behave identically.
   */
  public static void countdown(AddressableLEDBuffer buffer, double t, Color color, int chunks) {
    // note the integer division here, it automatically
    // rounds down in case the buffer length is not directly
    // divisible by the specified number of chunks
    final var chunkLength = buffer.getLength() / chunks;

    // number of LEDs in each chunk to enable
    final var visibleLength =
        MathUtil.clamp(chunkLength - (int) Math.round(chunkLength * t), 0, chunkLength);

    for (var i = 0; i < chunks; ++i) {
      for (var j = 0; j < chunkLength; ++j) {
        buffer.setLED(i * chunkLength + j, j < visibleLength ? color : Color.kBlack);
      }
    }
  }

  /**
   * Implements countup LED pattern with the solid color.
   *
   * <p>The implementation assumes that LED strip is split into a set of equally sized chunks, that
   * should each behave identically.
   */
  public static void countup(AddressableLEDBuffer buffer, double t, Color color, int chunks) {
    // note the integer division here, it automatically
    // rounds down in case the buffer length is not directly
    // divisible by the specified number of chunks
    final var chunkLength = buffer.getLength() / chunks;

    // number of LEDs in each chunk to enable
    final var visibleLength = MathUtil.clamp((int) Math.round(chunkLength * t), 0, chunkLength);

    for (var i = 0; i < chunks; ++i) {
      for (var j = 0; j < chunkLength; ++j) {
        buffer.setLED(i * chunkLength + j, j < visibleLength ? color : Color.kBlack);
      }
    }
  }

  /**
   * Implements "breath" LED pattern with the solid HSV color, where only value (brightness) is
   * animated.
   *
   * <p>The implementation follows trigonometric (sine) profile. LED brightness oscillates between
   * min and max that must be within [0-1] range.
   */
  public static void breath(
      AddressableLEDBuffer buffer, double t, ColorHSV color, double min, double max) {
    final var a = Math.max(0, Math.sin(Math.PI * t));
    final var v = MathUtil.clamp(min + a * (max - min), 0, 1);

    for (var i = 0; i < buffer.getLength(); ++i) {
      buffer.setHSV(i, color.hue(), color.saturation(), (int) Math.round(v * 255));
    }
  }

  /**
   * Implements "breath" LED pattern with the solid RGB color, where each color channel is animated.
   *
   * <p>The implementation follows trigonometric (sine) profile. LED brightness oscillates between
   * min and max that must be within [0-1] range.
   */
  public static void breath(
      AddressableLEDBuffer buffer, double t, Color color, double min, double max) {
    final var a = Math.max(0, Math.sin(Math.PI * t));
    final var v = MathUtil.clamp(min + a * (max - min), 0, 1);
    final var r = v * color.red;
    final var g = v * color.green;
    final var b = v * color.blue;

    for (var i = 0; i < buffer.getLength(); ++i) {
      buffer.setRGB(
          i, (int) Math.round(r * 255), (int) Math.round(g * 255), (int) Math.round(b * 255));
    }
  }

  private static final Random rnd = new Random();

  /**
   * Implements "random" LED pattern with the solid HSV color.
   *
   * <p>The implementation randomly varies brightness while retaining hue and saturation.
   */
  public static void random(AddressableLEDBuffer buffer, double t, ColorHSV color) {
    for (var i = 0; i < buffer.getLength(); ++i) {
      buffer.setHSV(i, color.hue(), color.saturation(), rnd.nextBoolean() ? 255 : 0);
    }
  }
}
