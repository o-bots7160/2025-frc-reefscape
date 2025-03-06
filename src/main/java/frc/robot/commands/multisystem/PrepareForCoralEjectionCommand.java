package frc.robot.commands.multisystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class PrepareForCoralEjectionCommand extends ParallelCommandGroup {
    public PrepareForCoralEjectionCommand(Command moveElevatorCommand, Command moveShoulderCommand) {
        super(moveElevatorCommand, moveShoulderCommand);
    }
}
