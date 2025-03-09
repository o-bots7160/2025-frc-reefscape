package frc.robot.commands.manipulator.coral;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ClearElevatorCommand;
import frc.robot.commands.elevator.MoveElevatorCommand;
import frc.robot.commands.manipulator.shoulder.RotateShoulderCommand;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class PlaceCoralCommand extends SequentialCommandGroup {

    public PlaceCoralCommand(
            // Subsystems
            //////////////////////////////////////////////////
            DriveBaseSubsystem driveBaseSubsystem,
            CoralIntakeSubsystem coralIntakeSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            ShoulderSubsystem shoulderSubsystem,
            // Suppliers
            //////////////////////////////////////////////////
            Supplier<Pose2d> coralReefPoseSupplier, Supplier<Double> coralLevelSupplier, Supplier<Double> coralLevelRotation) {
        super(
                // Ensure the elevator is clear before moving
                new ClearElevatorCommand(elevatorSubsystem),
                // Parallel commands to put robot in face position
                Commands.parallel(
                        // driveBaseSubsystem.moveTo(faceTarget),
                        new RotateShoulderCommand(shoulderSubsystem, coralLevelRotation),
                        new MoveElevatorCommand(elevatorSubsystem, coralLevelSupplier)));
    }

}
