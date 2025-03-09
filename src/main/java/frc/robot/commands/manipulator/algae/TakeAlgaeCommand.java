package frc.robot.commands.manipulator.algae;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ClearElevatorCommand;
import frc.robot.commands.elevator.MoveElevatorCommand;
import frc.robot.commands.manipulator.shoulder.RotateShoulderCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class TakeAlgaeCommand extends SequentialCommandGroup {

    public TakeAlgaeCommand(
            // Subsystems
            //////////////////////////////////////////////////
            DriveBaseSubsystem driveBaseSubsystem,
            AlgaeIntakeSubsystem algaeIntakeSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            ShoulderSubsystem shoulderSubsystem,
            // Suppliers
            //////////////////////////////////////////////////
            Supplier<Pose2d> algaeReefPoseSupplier,
            Supplier<Double> algaeLevelSupplier,
            Supplier<Double> algaeRotationRotation) {
        super(
                // Ensure the elevator is clear before moving
                new ClearElevatorCommand(elevatorSubsystem),
                // Parallel commands to put robot in face position
                Commands.parallel(
                        // driveBaseSubsystem.moveTo(algaeReefPoseSupplier.get()),
                        new RotateShoulderCommand(shoulderSubsystem, algaeRotationRotation),
                        new MoveElevatorCommand(elevatorSubsystem, algaeLevelSupplier)));
    }
}
