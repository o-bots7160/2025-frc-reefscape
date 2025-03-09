package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
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
}
