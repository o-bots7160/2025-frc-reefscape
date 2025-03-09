package frc.robot.commands.elevator;

import frc.robot.subsystems.ElevatorSubsystem;

public class ClearElevatorCommand extends MoveElevatorCommand {

    public ClearElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        super(elevatorSubsystem, null);
    }

    @Override
    public void initialize() {
        subsystem.setClear();
    }

    @Override
    public boolean isFinished() {
        return subsystem.isClear();
    }
}
