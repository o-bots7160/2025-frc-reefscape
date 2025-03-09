package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
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
}