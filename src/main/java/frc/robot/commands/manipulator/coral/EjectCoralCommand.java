package frc.robot.commands.manipulator.coral;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class EjectCoralCommand extends SequentialCommandGroup {

    // Constructor
    public EjectCoralCommand(CoralIntakeSubsystem coral) {
        super(coral.eject());
    }
}
