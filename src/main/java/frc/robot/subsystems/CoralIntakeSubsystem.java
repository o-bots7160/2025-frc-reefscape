package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
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
        return new CoralIntakeCommand(this, true);
    }

    public Command eject() {
        return new CoralIntakeCommand(this, false);
    }

}