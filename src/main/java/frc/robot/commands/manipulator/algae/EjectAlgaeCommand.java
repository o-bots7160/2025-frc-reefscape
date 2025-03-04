package frc.robot.commands.manipulator.algae;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TestLoggerCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class EjectAlgaeCommand extends SequentialCommandGroup {

    // Constructor
    public EjectAlgaeCommand(AlgaeIntakeSubsystem algae) {
        super(new TestLoggerCommand("Eject Algae"), algae.eject());
    }
}
