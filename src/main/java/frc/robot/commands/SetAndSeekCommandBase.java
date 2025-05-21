package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.AbstractSetAndSeekSubsystemConfig;
import frc.robot.helpers.Logger;
import frc.robot.subsystems.AbstractSetAndSeekSubsystem;

/**
 * A command that sets a target value for a subsystem and seeks to reach that target, working in conjunction with the {@link AbstractSetAndSeekSubsystem}
 * subsystems
 *
 * @param <T>       The type of the subsystem that extends SetAndSeekSubsystemBase.
 * @param <TConfig> The type of the subsystem configuration that extends SetAndSeekSubsystemConfigBase.
 */
public abstract class SetAndSeekCommandBase<T extends AbstractSetAndSeekSubsystem<TConfig>, TConfig extends AbstractSetAndSeekSubsystemConfig>
        extends Command {

    protected final T              subsystem;

    private final Supplier<Double> target;

    private Logger                 log = Logger.getInstance(this.getClass());

    private String                 subsystemName;

    public SetAndSeekCommandBase(T subsystem, double target) {
        this(subsystem, () -> target);
    }

    public SetAndSeekCommandBase(T subsystem, Supplier<Double> target) {
        this.target        = target;
        this.subsystem     = subsystem;
        this.subsystemName = subsystem.getName();
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        double t = target.get();
        log.info(subsystemName + ": Setting target to " + t);
        subsystem.setTarget(t);
    }

    @Override
    public void execute() {
        subsystem.seekTarget();
        double c = subsystem.getCurrentPosition();
        log.info(subsystemName + ": Seeking to target; currently at " + c);
    }

    @Override
    public boolean isFinished() {
        boolean atTarget = subsystem.atTarget();
        if (atTarget) {
            log.info(subsystemName + ": At target!");
        }
        return atTarget;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            double c = subsystem.getCurrentPosition();
            log.warning(subsystemName + ": Interrupted during seek; last position: " + c);
            subsystem.requestStop();
        } else {
            subsystem.stop();
        }
        log.info(subsystemName + ": Command ending.");
    }
}