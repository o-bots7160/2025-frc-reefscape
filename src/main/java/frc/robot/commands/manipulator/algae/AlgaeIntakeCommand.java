package frc.robot.commands.manipulator.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class AlgaeIntakeCommand extends Command {

    private boolean intake;

    private final AlgaeIntakeSubsystem subsystem;

    // Constructor
    public AlgaeIntakeCommand( AlgaeIntakeSubsystem new_subsystem, boolean new_intake ) {
        super();
        intake = new_intake;
        subsystem = new_subsystem;
        addRequirements( subsystem );
    }

    @Override
    public void initialize()
    {
        super.initialize();
        if (intake)
        {
            subsystem.setSpeed( -0.2 );
        }
        else
        {
            subsystem.setSpeed( 1.0 );
        }
    }

    @Override
    public boolean isFinished()
    {
        super.isFinished();
        if (intake)
        {
	    return subsystem.haveItem();
        }
        else
        {
	    return !subsystem.haveItem();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        subsystem.setSpeed( 0.0 );
    }
}


