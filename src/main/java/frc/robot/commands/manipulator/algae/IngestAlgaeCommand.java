package frc.robot.commands.manipulator.algae;

import frc.robot.commands.manipulator.IngestWithManipulatorCommandBase;
import frc.robot.config.AlgaeIntakeSubsystemConfig;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class IngestAlgaeCommand extends IngestWithManipulatorCommandBase<AlgaeIntakeSubsystem, AlgaeIntakeSubsystemConfig> {
    public IngestAlgaeCommand(AlgaeIntakeSubsystem algaeIntakeSubsystem) {
        super(algaeIntakeSubsystem);
    }
}
