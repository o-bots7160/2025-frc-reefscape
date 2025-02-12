package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

public class CoralIntakeCommand extends Command {

    private boolean intake;

    private final ManipulatorSubsystem subsystem;

    // Constructor
    public CoralIntakeCommand( ManipulatorSubsystem new_subsystem, boolean new_intake ) {
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
            subsystem.setCoral( 0.2 );
        }
        else
        {
            subsystem.setCoral( -0.2 );
        }
    }

    @Override
    public boolean isFinished()
    {
        super.isFinished();
        return subsystem.haveCoral();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        subsystem.setCoral( 0.0 );
    }
}


