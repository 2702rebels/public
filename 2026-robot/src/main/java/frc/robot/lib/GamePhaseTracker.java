package frc.robot.lib;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BooleanSupplier;

public final class GamePhaseTracker {
  public static enum Phase {
    Transition,
    Shift1,
    Shift2,
    Shift3,
    Shift4,
    EndGame,
    Auto,
    Disabled
  }

  public static enum AllianceOverride {
    None,
    Red,
    Blue
  }

  public record PhaseInfo(Phase phase, double elapsedTime, double remainingTime, boolean active) {}

  private static final double kAutoTime = 20;
  private static final double kTransitionTime = 10;
  private static final double kShift1Time = 35;
  private static final double kShift2Time = 60;
  private static final double kShift3Time = 85;
  private static final double kShift4Time = 110;
  private static final double kEndGameTime = 140;

  private static Timer timer = new Timer();

  private final SendableChooser<AllianceOverride> allianceAutoOverride = new SendableChooser<>();

  /** FMS (DS) didn't report the alliance winning auto and no override is selected. */
  public final BooleanSupplier missingAutoWinAlliance =
      () ->
          DriverStation.getGameSpecificMessage().length() <= 0
              && allianceAutoOverride.getSelected() == AllianceOverride.None;

  public GamePhaseTracker() {
    allianceAutoOverride.setDefaultOption("None", AllianceOverride.None);
    allianceAutoOverride.addOption("Red", AllianceOverride.Red);
    allianceAutoOverride.addOption("Blue", AllianceOverride.Blue);

    SmartDashboard.putData("Alliance Auto Win Override", allianceAutoOverride);
  }

  /**
   * Returns alliance to have their hub inactive first, i.e. alliance that WON in auto. Honors
   * manual override.
   */
  private Alliance getFirstInactiveAlliance() {
    final var allianceOverride = allianceAutoOverride.getSelected();
    if (allianceOverride != AllianceOverride.None) {
      return allianceOverride == AllianceOverride.Red ? Alliance.Red : Alliance.Blue;
    }

    final var message = DriverStation.getGameSpecificMessage();
    if (message.length() > 0) {
      switch (message.charAt(0)) {
        case 'B':
          return Alliance.Blue;
        case 'R':
          return Alliance.Red;
          // ignore invalid data
      }
    }

    // fallback default
    return Alliance.Blue;
  }

  /**
   * Returns the timestamp (seconds) since the start of the current period.
   *
   * <p>Can be switched to use FMS supplied time, instead of internal timer, for situations when the
   * robot code restarted during the match.
   */
  private double getPeriodTimestamp() {
    if (useFMStime.get()) {
      // match time returns _remaining_ time in the period,
      // we want _elapsed_ time instead
      final var matchTime = DriverStation.getMatchTime();
      final var period = DriverStation.isAutonomousEnabled() ? 20.0 : 140.0;
      return period - matchTime;
    }

    return timer.get();
  }

  /** Restarts the timer when next period starts. */
  public void startPeriod() {
    timer.restart();
  }

  /** Returns elapsed time of the current period. */
  public Time getPeriodElapsedTime() {
    return Seconds.of(getPeriodTimestamp());
  }

  /** Returns current phase information. */
  public PhaseInfo getPhase() {
    final var timestamp = getPeriodTimestamp();
    final var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    final var inactive = getFirstInactiveAlliance() == alliance;

    if (DriverStation.isAutonomousEnabled()) {
      return new PhaseInfo(Phase.Auto, timestamp, Math.max(0, kAutoTime - timestamp), true);
    }

    if (!DriverStation.isEnabled()) {
      return new PhaseInfo(Phase.Disabled, timestamp, 0, false);
    }

    if (timestamp < kTransitionTime) {
      return new PhaseInfo(Phase.Transition, timestamp, kTransitionTime - timestamp, true);
    }

    if (timestamp < kShift1Time) {
      return new PhaseInfo(
          Phase.Shift1, timestamp - kTransitionTime, kShift1Time - timestamp, !inactive);
    }

    if (timestamp < kShift2Time) {
      return new PhaseInfo(
          Phase.Shift2, timestamp - kShift1Time, kShift2Time - timestamp, inactive);
    }

    if (timestamp < kShift3Time) {
      return new PhaseInfo(
          Phase.Shift3, timestamp - kShift2Time, kShift3Time - timestamp, !inactive);
    }

    if (timestamp < kShift4Time) {
      return new PhaseInfo(
          Phase.Shift4, timestamp - kShift3Time, kShift4Time - timestamp, inactive);
    }

    return new PhaseInfo(
        Phase.EndGame, timestamp - kShift4Time, Math.max(0, kEndGameTime - timestamp), true);
  }

  private static final Tunable.BooleanValue useFMStime =
      Tunable.value("Control/UseFMSTime", false, true);
}
