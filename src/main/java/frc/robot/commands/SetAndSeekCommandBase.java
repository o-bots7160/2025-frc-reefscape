package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.SetAndSeekSubsystemConfigBase;
import frc.robot.subsystems.SetAndSeekSubsystemBase;

/**
 * A command that sets a target value for a subsystem and seeks to reach that target, working in conjunction with the {@link SetAndSeekSubsystemBase}
 * subsystems
 * 
 * @param <T>       The type of the subsystem that extends SetAndSeekSubsystemBase.
 * @param <TConfig> The type of the subsystem configuration that extends SetAndSeekSubsystemConfigBase.
 */
public abstract class SetAndSeekCommandBase<T extends SetAndSeekSubsystemBase<TConfig>, TConfig extends SetAndSeekSubsystemConfigBase>
        extends Command {

    protected final T              subsystem;

    private final Supplier<Double> target;

    public SetAndSeekCommandBase(T subsystem, double target) {
        this(subsystem, () -> target);
    }

    public SetAndSeekCommandBase(T subsystem, Supplier<Double> target) {
        this.target    = target;
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setTarget(target.get());
    }

    @Override
    public void execute() {
        subsystem.seekTarget();
    }

    @Override
    public boolean isFinished() {
        return subsystem.atTarget();
    }

    @Override
    public void end(boolean interrupted) {
        /*
         * // If we're interrupted, coast X distance as long as it's not beyond the original set point // otherwise, we're at the position and can
         * stop normally if (interrupted && !subsystem.atTarget()) { double originalTarget = target.get(); double current =
         * subsystem.getCurrentPosition(); boolean goingUp = originalTarget > current; double amountToMove = Math.abs(originalTarget - current) *
         * 0.25; double newDestination = goingUp ? current + amountToMove : current - amountToMove; if ((goingUp && newDestination > originalTarget)
         * || (!goingUp && newDestination < originalTarget)) { // we went too far! newDestination = originalTarget; }
         * subsystem.setTarget(newDestination); } else { subsystem.stop(); }
         */
        subsystem.stop();

    }
}