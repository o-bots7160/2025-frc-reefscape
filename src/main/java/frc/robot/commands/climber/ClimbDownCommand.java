package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.helpers.Logger;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbDownCommand extends Command {

    private Logger                 log = Logger.getInstance(this.getClass());

    private final ClimberSubsystem climberSubsystem;

    public ClimbDownCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        log.verbose("Initialized");
    }

    @Override
    public void execute() {
        log.verbose("Executing");
        climberSubsystem.start(-0.5);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrDownted) {
        climberSubsystem.stop();
    }
}