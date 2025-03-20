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
        if (interrupted) {
            subsystem.slowStop();
        } else {
            subsystem.stop();
        }
    }
}