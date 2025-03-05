package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * This command checks if the elevator is clear and sets it to clear if it is not. It then seeks the target position and finishes when the target is
 * reached.
 * 
 * @param elevatorSubsystem The elevator subsystem to be controlled by this command.
 */
public class ClearElevatorCommand extends Command {

    private ElevatorSubsystem subsystem;

    public ClearElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        this.subsystem = elevatorSubsystem;

    }

    @Override
    public void initialize() {
        super.initialize();

        boolean isClear = subsystem.isClear();
        if (!isClear) {
            subsystem.setClear();
        }
    }

    @Override
    public void execute() {
        subsystem.seekTarget();
    }

    @Override
    public boolean isFinished() {
        // checking for both, because if we go past clear but haven't technically hit the target, we still want to end
        return subsystem.atTarget() || subsystem.isClear();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }
}
