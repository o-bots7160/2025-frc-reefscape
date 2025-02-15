package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class CollectCoralCommand extends SequentialCommandGroup {

    // Constructor
    public CollectCoralCommand ( DriveBaseSubsystem   drive,
                                 CoralIntakeSubsystem coral, 
                                 ElevatorSubsystem    elevator,
                                 ShoulderSubsystem    shoulder,
                                 Pose2d               faceTarget,
                                 Pose2d               coralTarget,
                                 double               levelTarget ) {
        super( Commands.parallel( drive.moveTo( faceTarget ),
                                 elevator.goToCommand( levelTarget ),
                                 shoulder.shoulderCommand( 0.0 ) ),
               drive.moveTo( coralTarget ),
               coral.inject( ),
               Commands.parallel( drive.moveTo( faceTarget ),
                                 elevator.goToCommand( 0.0 ),
                                 shoulder.shoulderCommand( 0.0 ) ) );
    }
}


