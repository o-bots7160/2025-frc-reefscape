package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ClearElevatorCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;

    public ClearElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        super();
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setClear();
    }

    @Override
    public void execute() {
        elevatorSubsystem.seekTarget();
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.isClear();
    }
}
