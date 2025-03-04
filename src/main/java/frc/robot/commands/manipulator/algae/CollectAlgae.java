package frc.robot.commands.manipulator.algae;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TestLoggerCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class CollectAlgae extends SequentialCommandGroup {

    // Constructor
    public CollectAlgae ( DriveBaseSubsystem   drive,
                                 AlgaeIntakeSubsystem algae, 
                                 ElevatorSubsystem    elevator,
                                 ShoulderSubsystem    shoulder,
                                 Pose2d               faceTarget,
                                 Pose2d               algaeTarget ) {
        super( new TestLoggerCommand("Collect Algae Direct"),
               Commands.parallel( drive.moveTo( faceTarget ),
                                 elevator.goToCommand( 0.0 ),
                                 shoulder.shoulderCommand( 0.0 )),
               drive.moveTo( algaeTarget ),
               algae.inject( ),
               Commands.parallel( drive.moveTo( faceTarget ),
                                 elevator.goToCommand( 0.0 ),
                                 shoulder.shoulderCommand( 0.0 ) ) );
    }
    // Constructor
    public CollectAlgae ( DriveBaseSubsystem   drive,
                                 AlgaeIntakeSubsystem algae, 
                                 ElevatorSubsystem    elevator,
                                 ShoulderSubsystem    shoulder,
                                 Supplier<Pose2d>     faceTarget,
                                 Supplier<Pose2d>     algaeTarget ) {
        super( new TestLoggerCommand("Collect Algae Supplier"),
               Commands.parallel( drive.moveTo( faceTarget.get() ),
                                 elevator.goToCommand( 0.0 ),
                                 shoulder.shoulderCommand( 0.0 ) ),
               drive.moveTo( algaeTarget.get() ),
               algae.inject( ),
               Commands.parallel( drive.moveTo( faceTarget ),
                                 elevator.goToCommand( 0.0 ),
                                 shoulder.shoulderCommand( 0.0 ) ) );
    }
}


