package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class EjectCoralCommand extends SequentialCommandGroup {

    // Constructor
    public EjectCoralCommand ( CoralIntakeSubsystem   coral,
                                 ShoulderSubsystem    shoulder) {
        super( new TestLoggerCommand("Eject Coral"),
               shoulder.shoulderCommand( 0.0 ),
               coral.eject(),
               shoulder.shoulderCommand( 0.0 ) );
    }
}


