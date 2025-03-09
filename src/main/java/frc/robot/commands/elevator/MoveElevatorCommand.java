package frc.robot.commands.elevator;

import java.util.function.Supplier;

import frc.robot.commands.SetAndSeekCommandBase;
import frc.robot.config.ElevatorSubsystemConfig;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Command to move the elevator to a specified target position. This command uses a target position supplied either as a fixed value or dynamically
 * via a Supplier.
 * 
 * @param elevatorSubsystem The subsystem used by this command.
 * @param target            The target position for the elevator.
 * @param targetSupplier    A supplier that provides the target position for the elevator.
 */
public class MoveElevatorCommand extends SetAndSeekCommandBase<ElevatorSubsystem, ElevatorSubsystemConfig> {

    public MoveElevatorCommand(ElevatorSubsystem elevatorSubsystem, double target) {
        super(elevatorSubsystem, target);
    }

    public MoveElevatorCommand(ElevatorSubsystem elevatorSubsystem, Supplier<Double> target) {
        super(elevatorSubsystem, target);
    }
}
