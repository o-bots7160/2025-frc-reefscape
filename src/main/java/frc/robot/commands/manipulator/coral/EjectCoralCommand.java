package frc.robot.commands.manipulator.coral;

import frc.robot.commands.manipulator.EjectWithManipulatorCommandBase;
import frc.robot.config.CoralIntakeSubsystemConfig;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class EjectCoralCommand extends EjectWithManipulatorCommandBase<CoralIntakeSubsystem, CoralIntakeSubsystemConfig> {

    // Constructor
    public EjectCoralCommand(CoralIntakeSubsystem coralIntakeSubsystem) {
        super(coralIntakeSubsystem);
    }
}
