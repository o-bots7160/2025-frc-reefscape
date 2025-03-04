package frc.robot.commands.manipulator.coral;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class PlaceCoralCommand extends SequentialCommandGroup {

    // Constructor
    public PlaceCoralCommand(
            // Subsystems
            //////////////////////////////////////////////////
            DriveBaseSubsystem drive, CoralIntakeSubsystem coral, ElevatorSubsystem elevator, ShoulderSubsystem shoulder,
            // Suppliers
            //////////////////////////////////////////////////
            java.util.function.Supplier<Pose2d> faceTarget, java.util.function.Supplier<Pose2d> reefTarget,
            java.util.function.Supplier<Double> levelTarget) {
        super(
                // Parallel commands to put robot in face position
                Commands.parallel(drive.moveTo(faceTarget), elevator.goToCommand(levelTarget), shoulder.shoulderCommand(0.0)),
                // Align with target and eject
                drive.moveTo(reefTarget));
    }

}
