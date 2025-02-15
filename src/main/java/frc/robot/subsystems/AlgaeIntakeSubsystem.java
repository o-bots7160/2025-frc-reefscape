package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.manipulator.AlgaeIntakeCommand;
import frc.robot.config.AlgaeIntakeSubsystemConfig;

/**
 *
 */
@Logged
public class AlgaeIntakeSubsystem extends IntakeSubsystemBase<AlgaeIntakeSubsystemConfig> {

    /**
    *
    */
    public AlgaeIntakeSubsystem() {

    }

    public Command intakeCommand(boolean new_intake) {
        return new AlgaeIntakeCommand(this, new_intake);
    }

    @Override
    protected AlgaeIntakeSubsystemConfig getConfig() {
        return subsystemsConfig.algaeIntakeSubsystem;
    }
}