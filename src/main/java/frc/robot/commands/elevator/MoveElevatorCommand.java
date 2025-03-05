package frc.robot.commands.elevator;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.helpers.Logger;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 *
 */
@Logged
public class MoveElevatorCommand extends Command {
    private final Supplier<Double>  targetSupplier;

    private final ElevatorSubsystem subsystem;

    private Logger                  log = Logger.getInstance(this.getClass());

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
        double target = targetSupplier.get();
        subsystem.setTarget(target);
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
