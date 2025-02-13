package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 *
 */
@Logged
public class ElevatorCommand extends Command {
    private final double            target;
    private final ElevatorSubsystem subsystem;

    public ElevatorCommand( ElevatorSubsystem new_subsystem, double new_target ) {
        super();
        target    = new_target;
        subsystem = new_subsystem;
        addRequirements( subsystem );
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        super.initialize();
        subsystem.setTarget( target );
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return subsystem.atTarget( );
    }
}
