package frc.robot.commands.manipulator.shoulder;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.helpers.Logger;
import frc.robot.subsystems.ShoulderSubsystem;

public class RotateShoulderCommand extends Command {
    private Logger                  log = Logger.getInstance(this.getClass());

    private final Supplier<Double>  target;

    private final ShoulderSubsystem shoulderSubsystem;

    // Constructor
    public RotateShoulderCommand(ShoulderSubsystem shoulderSubsystem, double target) {
        this(shoulderSubsystem, () -> target);
    }

    // Constructor
    public RotateShoulderCommand(ShoulderSubsystem shoulderSubsystem, Supplier<Double> target) {
        this.target            = target;
        this.shoulderSubsystem = shoulderSubsystem;
        addRequirements(shoulderSubsystem);
    }

    @Override
    public void initialize() {
        shoulderSubsystem.setTarget(target.get());
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