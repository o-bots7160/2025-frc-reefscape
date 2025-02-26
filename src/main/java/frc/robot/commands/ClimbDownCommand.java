package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.helpers.Logger;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbDownCommand extends Command {

    private Logger                 log = Logger.getInstance(this.getClass());

    private final ClimberSubsystem subsystem;

    public ClimbDownCommand(ClimberSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // subsystem.setTarget(target);
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