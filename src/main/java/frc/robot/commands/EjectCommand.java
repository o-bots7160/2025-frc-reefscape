package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class EjectCommand extends SequentialCommandGroup {

    // Constructor
    public EjectCommand ( CoralIntakeSubsystem   coral,
                                 AlgaeIntakeSubsystem algae, 
                                 ElevatorSubsystem    elevator,
                                 ShoulderSubsystem    shoulder) {
        super( new TestLoggerCommand("Eject Game Piece"),
               Commands.parallel( elevator.goToCommand( 0.0 ),
                                 shoulder.shoulderCommand( 0.0 )),
               Commands.parallel( elevator.goToCommand( 0.0 ),
                                 shoulder.shoulderCommand( 0.0 ) ) );
    }
}


