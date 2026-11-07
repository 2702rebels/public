package frc.robot.lib;

import edu.wpi.first.math.MathShared;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public final class Hack {
  public static final double kLoopOverrunWatchdogSeconds = 0.2;

  /** Adjusts loop overrun warnings to a longer period. */
  public static void adjustLoopOverrun(frc.robot.Robot robot) {
    try {
      final var watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      final var watchdog = (Watchdog) watchdogField.get(robot);
      watchdog.setTimeout(kLoopOverrunWatchdogSeconds);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
    }

    CommandScheduler.getInstance().setPeriod(kLoopOverrunWatchdogSeconds);
  }

  /** Silence various warnings. */
  public static void silenceWarnings() {
    DriverStation.silenceJoystickConnectionWarning(true);

    var mathShared = MathSharedStore.getMathShared();
    MathSharedStore.setMathShared(
        new MathShared() {
          @Override
          public void reportError(String error, StackTraceElement[] stackTrace) {
            if (error.startsWith("x and y components of Rotation2d are zero")) {
              return;
            }
            mathShared.reportError(error, stackTrace);
          }

          @Override
          public void reportUsage(MathUsageId id, int count) {
            mathShared.reportUsage(id, count);
          }

          @Override
          public double getTimestamp() {
            return mathShared.getTimestamp();
          }
        });
  }
}
