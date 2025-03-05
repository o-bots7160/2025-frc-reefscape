package frc.robot.commands.elevator;

import frc.robot.subsystems.ElevatorSubsystem;

public class StowElevatorCommand extends MoveElevatorCommand {
    private final static double stowPosition = 0.0;

    public StowElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        super(elevatorSubsystem, stowPosition);

    }

}
