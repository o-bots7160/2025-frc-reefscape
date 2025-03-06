package frc.robot.commands.manipulator.shoulder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.helpers.Logger;
import frc.robot.subsystems.ShoulderSubsystem;

public class RotateShoulderCommand extends Command {
    private Logger                  log = Logger.getInstance(this.getClass());

    private final double            target;

    private final ShoulderSubsystem shoulderSubsystem;

    // Constructor
    public RotateShoulderCommand(ShoulderSubsystem shoulderSubsystem, double target) {
        this.target            = target;
        this.shoulderSubsystem = shoulderSubsystem;
        addRequirements(shoulderSubsystem);
    }

    @Override
    public void initialize() {
        shoulderSubsystem.setTarget(target);
        log.verbose("Target: " + target);
    }

    @Override
    public void execute() {
        super.execute();
        shoulderSubsystem.seekTarget();
    }

    @Override
    public boolean isFinished() {
        return shoulderSubsystem.atTarget();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        shoulderSubsystem.stop();
    }
}