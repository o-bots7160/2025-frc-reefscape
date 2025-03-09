package frc.robot.commands.manipulator;

import java.util.function.Supplier;

import frc.robot.commands.SetAndSeekCommandBase;
import frc.robot.config.ShoulderSubsystemConfig;
import frc.robot.subsystems.ShoulderSubsystem;

public class RotateShoulderCommand extends SetAndSeekCommandBase<ShoulderSubsystem, ShoulderSubsystemConfig> {
    public RotateShoulderCommand(ShoulderSubsystem shoulderSubsystem, Supplier<Double> target) {
        super(shoulderSubsystem, target);
    }

    public RotateShoulderCommand(ShoulderSubsystem shoulderSubsystem, double target) {
        super(shoulderSubsystem, target);
    }
}
