package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * An instant command that implements requirements of the default command expected by the scheduler;
 * it will initialize, execute once, and never end, unless interrupted.
 *
 * <p>Use this command to wrap subsystem operations that you may want to use as the default command
 * for the subsystem or to create fire-and-forget "forever" commands, e.g. can be used with LED
 * subsystem to switch modes.
 */
public class DefaultCommand extends FunctionalCommand {
  /**
   * Creates a new DefaultCommand that runs the given Runnable with the given requirements.
   *
   * @param toRun Runnable to run
   * @param requirements Subsystems required by this command
   */
  public DefaultCommand(Runnable toRun, Subsystem... requirements) {
    super(toRun, () -> {}, interrupted -> {}, () -> false, requirements);
  }
}
