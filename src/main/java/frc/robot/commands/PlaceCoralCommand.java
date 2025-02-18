package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class PlaceCoralCommand extends SequentialCommandGroup {

    // Constructor
    public PlaceCoralCommand ( DriveBaseSubsystem   drive,
                               CoralIntakeSubsystem coral, 
                               ElevatorSubsystem    elevator,
                               ShoulderSubsystem    shoulder,
                               Pose2d               faceTarget,
                               Pose2d               reefTarget,
                               double               levelTarget ) {
        super( Commands.parallel( drive.moveTo( faceTarget ),
                                  elevator.goToCommand( levelTarget ),
                                  shoulder.shoulderCommand( 0.0 ) ),
               drive.moveTo( reefTarget ),
               coral.eject( ),
               drive.moveTo( faceTarget ),
               Commands.parallel( elevator.goToCommand( 0.0 ),
                                  shoulder.shoulderCommand( 0.0 ) ) );
    }
    public PlaceCoralCommand ( DriveBaseSubsystem   drive,
                               CoralIntakeSubsystem coral, 
                               ElevatorSubsystem    elevator,
                               ShoulderSubsystem    shoulder,
                               Supplier<Pose2d>     faceTarget,
                               Supplier<Pose2d>     reefTarget,
                               Supplier<Double>     levelTarget ) {
        super( Commands.parallel( drive.moveTo( faceTarget ),
                                  elevator.goToCommand( levelTarget ),
                                  shoulder.shoulderCommand( 0.0 ) ),
               drive.moveTo( reefTarget ),
               coral.eject( ),
               drive.moveTo( faceTarget ),
               Commands.parallel( elevator.goToCommand( 0.0 ),
                                  shoulder.shoulderCommand( 0.0 ) ) );
    }
}


