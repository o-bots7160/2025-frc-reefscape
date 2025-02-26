package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class TravelCommand extends SequentialCommandGroup {

    // Constructor
    public TravelCommand ( ElevatorSubsystem    elevator,
                                ShoulderSubsystem    shoulder ) {
        super( new TestLoggerCommand("Travel"),
               Commands.parallel( elevator.goToCommand( 6.0 ),
                                  shoulder.shoulderCommand( 0.0 )));
    }
}


