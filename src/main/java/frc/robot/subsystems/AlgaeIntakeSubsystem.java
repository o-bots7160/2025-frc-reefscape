package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TestLoggerCommand;
import frc.robot.commands.manipulator.algae.AlgaeIntakeCommand;
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
        super(subsystemsConfig.algaeIntakeSubsystem);

    }

    public Command eject() {
        if (checkDisabled()) {
            return new TestLoggerCommand("eject method not called");
        }

        return new AlgaeIntakeCommand(this, false);
    }

    public Command inject() {
        if (checkDisabled()) {
            return new TestLoggerCommand("inject method not called");
        }

        return new AlgaeIntakeCommand(this, true);
    }
}