package frc.robot.commands.elevator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.helpers.Logger;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Command to move the elevator to a specified target position. This command uses a target position supplied either as a fixed value or dynamically
 * via a Supplier.
 * 
 * @param elevatorSubsystem The subsystem used by this command.
 * @param target            The target position for the elevator.
 * @param targetSupplier    A supplier that provides the target position for the elevator.
 */
public class MoveElevatorCommand extends Command {
    private final Supplier<Double>  targetSupplier;

    private final ElevatorSubsystem subsystem;

    private Logger                  log = Logger.getInstance(this.getClass());

    public MoveElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        subsystem      = elevatorSubsystem;
        targetSupplier = null;
        addRequirements(subsystem);
    }

    public MoveElevatorCommand(ElevatorSubsystem elevatorSubsystem, double target) {
        this(elevatorSubsystem, () -> target);
    }

    public MoveElevatorCommand(ElevatorSubsystem elevatorSubsystem, Supplier<Double> targetSupplier) {
        this.targetSupplier = targetSupplier;
        subsystem           = elevatorSubsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        if ( targetSupplier != null )
        {
            double target = targetSupplier.get();
            subsystem.setTarget(target);
        }
        log.dashboardVerbose("State", "Initialized");
    }

    @Override
    public void execute() {
        subsystem.seekTarget();
        log.dashboardVerbose("State", "Executing");
    }

    @Override
    public boolean isFinished() {
        boolean atTarget = subsystem.atTarget();
        if (atTarget) {
            log.dashboardVerbose("State", "Initialized");
        }
        return atTarget;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
        log.dashboardVerbose("State", "End");
    }
}
