package frc.robot.commands;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.*;
import java.util.Map;
import java.util.function.Supplier;

/** Extends SelectCommand class with ability to dynamically update the command being executed */
public class DynamicSelectCommand<K> extends Command {
  private final Map<K, Command> commands;
  private final Supplier<? extends K> selector;
  private Command selectedCommand;
  private boolean runsWhenDisabled = true;
  private InterruptionBehavior interruptBehavior = InterruptionBehavior.kCancelIncoming;

  private final Command defaultCommand =
      new PrintCommand("DynamicSelectCommand selector value does not correspond to any command!");

  /**
   * Creates a new DynamicSelectCommand.
   *
   * @param commands Map of commands to choose from
   * @param selector Selector to determine which command to run
   */
  public DynamicSelectCommand(Map<K, Command> commands, Supplier<? extends K> selector) {
    this.commands = requireNonNullParam(commands, "commands", "DynamicSelectCommand");
    this.selector = requireNonNullParam(selector, "selector", "DynamicSelectCommand");

    CommandScheduler.getInstance().registerComposedCommands(defaultCommand);
    CommandScheduler.getInstance()
        .registerComposedCommands(commands.values().toArray(new Command[] {}));

    for (Command command : this.commands.values()) {
      addRequirements(command.getRequirements());
      runsWhenDisabled &= command.runsWhenDisabled();
      if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
        interruptBehavior = InterruptionBehavior.kCancelSelf;
      }
    }
  }

  private Command getNextCommand() {
    return commands.getOrDefault(selector.get(), defaultCommand);
  }

  @Override
  public void initialize() {
    selectedCommand = getNextCommand();
    selectedCommand.initialize();
  }

  @Override
  public void execute() {
    final var currentCommand = selectedCommand;
    final var nextCommand = getNextCommand();
    if (currentCommand.equals(nextCommand)) {
      currentCommand.execute();
    } else {
      currentCommand.cancel();
      nextCommand.initialize();
      nextCommand.execute();
    }
    selectedCommand = nextCommand;
  }

  @Override
  public void end(boolean interrupted) {
    selectedCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return selectedCommand.isFinished();
  }

  @Override
  public boolean runsWhenDisabled() {
    return runsWhenDisabled;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return interruptBehavior;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty(
        "selected", () -> selectedCommand == null ? "null" : selectedCommand.getName(), null);
  }
}
