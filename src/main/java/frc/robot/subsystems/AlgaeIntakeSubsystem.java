package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.manipulator.AlgaeIntakeCommand;
import frc.robot.config.AlgaeIntakeSubsystemConfig;
import frc.robot.config.SubsystemsConfig;

/**
 *
 */
@Logged
public class AlgaeIntakeSubsystem extends IntakeSubsystemBase<AlgaeIntakeSubsystemConfig> {

    /**
    *
    */
    public AlgaeIntakeSubsystem(SubsystemsConfig subsystemsConfig) {
        super(subsystemsConfig);

    }

    public Command eject() {
        return new AlgaeIntakeCommand(this, false);
    }

    public Command inject() {
        return new AlgaeIntakeCommand(this, true);
    }

    @Override
    protected AlgaeIntakeSubsystemConfig getConfig() {
        return subsystemsConfig.algaeIntakeSubsystem;
    }
}