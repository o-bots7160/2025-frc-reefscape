package frc.robot.commands.elevator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * This command checks if the elevator is clear and sets it to clear if it is not. It then seeks the target position and finishes when the target is
 * reached.
 * 
 * @param elevatorSubsystem The elevator subsystem to be controlled by this command.
 */
public class MoveElevatorToRangeCommand extends Command {

    private ElevatorSubsystem elevatorSubsystem;

    private Runnable          targetSetter;

    private Supplier<Boolean> targetConfirmer;

    public MoveElevatorToRangeCommand(ElevatorSubsystem elevatorSubsystem, Runnable targetSetter, Supplier<Boolean> targetConfirmer) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.targetSetter      = targetSetter;
        this.targetConfirmer   = targetConfirmer;

    }

    @Override
    public void initialize() {
        super.initialize();

        boolean isClear = targetConfirmer.get();
        if (!isClear) {
            targetSetter.run();
        }
    }

    @Override
    public void execute() {
        elevatorSubsystem.seekTarget();
    }

    @Override
    public boolean isFinished() {
        // checking for both, because if we go past clear but haven't technically hit the target, we still want to end
        return elevatorSubsystem.atTarget() || targetConfirmer.get();
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stop();
    }
}
