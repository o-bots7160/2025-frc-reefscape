package frc.robot.commands.manipulator.coral;

import frc.robot.commands.manipulator.IngestWithManipulatorCommandBase;
import frc.robot.config.CoralIntakeSubsystemConfig;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class IngestCoralCommand extends IngestWithManipulatorCommandBase<CoralIntakeSubsystem, CoralIntakeSubsystemConfig> {

    public IngestCoralCommand(CoralIntakeSubsystem coralIntakeSubsystem) {
        super(coralIntakeSubsystem);
    }
}
