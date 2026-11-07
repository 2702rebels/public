package frc.robot.commands;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.function.BooleanSupplier;

/** A command that runs another command repeatedly as long as a condition is met. */
public class WhileCommand extends Command {
  private final Command command;
  private final BooleanSupplier supplier;
  private boolean ended;
  private boolean whileEnded;

  /**
   * Creates a new command that keeps executing the specified command while the supplied condition
   * is true.
   *
   * @param command Command to run repeatedly
   */
  public WhileCommand(Command command, BooleanSupplier supplier) {
    this.command = requireNonNullParam(command, "command", "WhileCommand");
    this.supplier = requireNonNullParam(supplier, "supplier", "WhileCommand");

    CommandScheduler.getInstance().registerComposedCommands(command);
    addRequirements(command.getRequirements());

    setName("While(" + command.getName() + ")");
  }

  @Override
  public void initialize() {
    ended = true;
    whileEnded = false;
  }

  @Override
  public void execute() {
    if (whileEnded) {
      return;
    }

    if (ended) {
      if (!supplier.getAsBoolean()) {
        whileEnded = true;
        return;
      }

      ended = false;
      command.initialize();
    }

    command.execute();

    if (command.isFinished()) {
      // restart command
      command.end(false);
      ended = true;
    }
  }

  @Override
  public boolean isFinished() {
    return whileEnded;
  }

  @Override
  public void end(boolean interrupted) {
    // make sure we didn't already call end(),
    // which would happen if the command finished in the last call to our execute()
    if (!ended) {
      command.end(interrupted);
      ended = true;
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return command.runsWhenDisabled();
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return command.getInterruptionBehavior();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("command", command::getName, null);
  }
}
