package frc.robot.commands.elevator;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 *
 */
@Logged
public class ElevatorCommand extends Command {
    private double                  target;

    private final Supplier<Double>  targetSupplier;

    private final ElevatorSubsystem subsystem;

    public ElevatorCommand(ElevatorSubsystem new_subsystem, double new_target) {
        super();
        target         = new_target;
        targetSupplier = null;
        subsystem      = new_subsystem;
        addRequirements(subsystem);
    }

    public ElevatorCommand(ElevatorSubsystem new_subsystem, Supplier<Double> new_target) {
        super();
        target         = 0.0;
        targetSupplier = new_target;
        subsystem      = new_subsystem;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        super.initialize();
        if (targetSupplier != null) {
            target = targetSupplier.get();
        }
        subsystem.setTarget(target);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return subsystem.atTarget();
    }
}
