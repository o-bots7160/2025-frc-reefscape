package frc.robot.commands.elevator;

import frc.robot.subsystems.ElevatorSubsystem;

public class ClearElevatorCommand extends MoveElevatorCommand {
    private ElevatorSubsystem elevatorSubsystem;

    public ClearElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        super(elevatorSubsystem, null);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setClear();
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.isClear();
    }
}
