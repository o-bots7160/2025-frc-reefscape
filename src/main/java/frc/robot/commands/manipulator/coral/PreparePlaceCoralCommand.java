package frc.robot.commands.manipulator.coral;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ClearElevatorCommand;
import frc.robot.commands.elevator.MoveElevatorCommand;
import frc.robot.commands.manipulator.RotateShoulderCommand;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class PreparePlaceCoralCommand extends SequentialCommandGroup {

    public PreparePlaceCoralCommand(
            CoralIntakeSubsystem coralIntakeSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            ShoulderSubsystem shoulderSubsystem,
            // Suppliers
            //////////////////////////////////////////////////
            Supplier<Double> coralLevel, Supplier<Double> coralLevelRotation) {
        super(
                // Ensure the elevator is clear before moving
                new ClearElevatorCommand(elevatorSubsystem).unless(() -> elevatorSubsystem.isClear()),
                // Parallel commands to put robot in face position
                Commands.parallel(
                        new RotateShoulderCommand(shoulderSubsystem, coralLevelRotation),
                        new MoveElevatorCommand(elevatorSubsystem, coralLevel)));
    }

}
