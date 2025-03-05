package frc.robot.commands.elevator;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 *
 */
@Logged
public class MoveElevatorCommand extends Command {
    private final Supplier<Double>  targetSupplier;

    private final ElevatorSubsystem subsystem;

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
        subsystem.stop();
    }
}
