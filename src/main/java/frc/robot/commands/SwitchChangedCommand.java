package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command that triggers a specified action when a switch state changes.
 * <p>
 * Executes the provided action with {@code true} when the command is executed, and with {@code false} when the command finishes.
 */
public class SwitchChangedCommand extends Command {
    private final java.util.function.Consumer<Boolean> switchChangedAction;

    public SwitchChangedCommand(java.util.function.Consumer<Boolean> switchChangedAction) {
        this.switchChangedAction = switchChangedAction;
    }

    @Override
    public void initialize() {
        switchChangedAction.accept(true);
    }

    @Override
    public void end(boolean interrupted) {
        switchChangedAction.accept(false);
    }

}
