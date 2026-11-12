package frc.robot.util;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;

public class AngleSlewRateLimiter {
  private double rateLimit;
  private double prevVal;
  private double prevTime;

  public AngleSlewRateLimiter(double rateLimit, double initialValue) {
    this.rateLimit = rateLimit;
    this.prevVal = initialValue;
    this.prevTime = MathSharedStore.getTimestamp();
  }

  public AngleSlewRateLimiter(double rateLimit) {
    this(rateLimit, 0);
  }

  public double calculate(double input) {
    return calculate(input, 1.0);
  }

  public double calculate(double input, double factor) {
    final var currentTime = MathSharedStore.getTimestamp();
    final var elapsedTime = currentTime - prevTime;
    final var delta = Misc.shortestArc(prevVal, input);

    // factor here allows adjusting effective rate at runtime
    final var step = rateLimit * elapsedTime * factor;

    if (Math.abs(delta) < step) {
      prevVal = Misc.clampAngle(input);
    } else {
      prevVal = Misc.clampAngle(prevVal + MathUtil.clamp(delta, -step, step));
    }

    prevTime = currentTime;
    return prevVal;
  }

  /** Instantly resets the slew rate limiter to the specified value. */
  public void Reset(double value) {
    prevVal = Misc.clampAngle(value);
    prevTime = MathSharedStore.getTimestamp();
  }

  /** Sets new rate limits. */
  public void setLimit(double rateLimit) {
    this.rateLimit = rateLimit;
  }
}
