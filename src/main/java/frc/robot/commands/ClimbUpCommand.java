package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.helpers.Logger;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbUpCommand extends Command {

    private Logger                 log = Logger.getInstance(this.getClass());

    private final double           target;

    private final ClimberSubsystem subsystem;

    public ClimbUpCommand(ClimberSubsystem subsystem, double target) {
        this.target    = target;
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // subsystem.setTarget(target);
        log.verbose("Target: " + target);
    }

    @Override
    public void execute() {
        super.execute();
        // subsystem.seekTarget();
    }

    @Override
    public boolean isFinished() {
        // return subsystem.atTarget();
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        subsystem.stop();
    }
}