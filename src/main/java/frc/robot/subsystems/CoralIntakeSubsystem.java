package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.manipulator.coral.CoralIntakeCommand;
import frc.robot.config.CoralIntakeSubsystemConfig;
import frc.robot.config.SubsystemsConfig;

/**
 *
 */
@Logged
public class CoralIntakeSubsystem extends IntakeSubsystemBase<CoralIntakeSubsystemConfig> {
    public CoralIntakeSubsystem(SubsystemsConfig subsystemsConfig) {
        super(subsystemsConfig.coralIntakeSubsystem);
    }

    public Command inject() {
        if (checkDisabled()) {
            return new InstantCommand(() -> log.warning("eject method not called"));
        }

        return new CoralIntakeCommand(this, true);
    }

    public Command eject() {
        if (checkDisabled()) {
            return new InstantCommand(() -> log.warning("inject method not called"));
        }

        return new CoralIntakeCommand(this, false);
    }

}