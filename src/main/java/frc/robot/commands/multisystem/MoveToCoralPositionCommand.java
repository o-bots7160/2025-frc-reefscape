package frc.robot.commands.multisystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MoveToCoralPositionCommand extends SequentialCommandGroup {
    public MoveToCoralPositionCommand(Command clearElevatorCommand, Command moveElevatorCommand, Command rotateShoulderCommand) {
        super(clearElevatorCommand, new ParallelCommandGroup(moveElevatorCommand, rotateShoulderCommand));
    }
}
