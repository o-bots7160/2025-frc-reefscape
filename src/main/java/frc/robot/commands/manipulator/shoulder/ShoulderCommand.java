package frc.robot.commands.manipulator.shoulder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.helpers.Logger;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderCommand extends Command {
    private Logger                  log = Logger.getInstance(this.getClass());

    private final double            target;

    private final ShoulderSubsystem subsystem;

    // Constructor
    public ShoulderCommand(ShoulderSubsystem new_subsystem, double new_target) {
        target    = new_target;
        subsystem = new_subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setTarget(target);
        log.verbose("Target: " + target);
    }

    @Override
    public void execute() {
        super.execute();
        subsystem.seekTarget();
    }

    @Override
    public boolean isFinished() {
        return subsystem.atTarget();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        subsystem.stop(); // maybe hold()
    }
}