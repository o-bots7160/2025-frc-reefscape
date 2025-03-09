package frc.robot.commands.manipulator.algae;

import frc.robot.commands.manipulator.EjectWithManipulatorCommandBase;
import frc.robot.config.AlgaeIntakeSubsystemConfig;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class EjectAlgaeCommand extends EjectWithManipulatorCommandBase<AlgaeIntakeSubsystem, AlgaeIntakeSubsystemConfig> {
    public EjectAlgaeCommand(AlgaeIntakeSubsystem algaeIntakeSubsystem) {
        super(algaeIntakeSubsystem);
    }
}
