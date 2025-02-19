package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderCommand extends Command {
    private final double            target;
    private final ShoulderSubsystem subsystem;

    // Constructor
    public ShoulderCommand( ShoulderSubsystem new_subsystem, double new_target ) {
        super();
        target    = new_target;
        subsystem = new_subsystem;
        addRequirements( subsystem );
    }

    @Override
    public void initialize()
    {
        super.initialize();
        subsystem.setTarget( target );
        System.out.println("Target: " + target);
    }

    @Override
    public boolean isFinished()
    {
        super.isFinished();
        return subsystem.atTarget();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        // TODO: Anything to do here? it doesn't seem like it.
    }
}