package frc.robot.commands.manipulator.algae;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ClearElevatorCommand;
import frc.robot.commands.elevator.MoveElevatorCommand;
import frc.robot.commands.manipulator.RotateShoulderCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class PrepareTakeAlgaeCommand extends SequentialCommandGroup {

    public PrepareTakeAlgaeCommand(
            AlgaeIntakeSubsystem algaeIntakeSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            ShoulderSubsystem shoulderSubsystem,
            Supplier<Double> algaeLevelSupplier,
            Supplier<Double> algaeRotationRotation) {
        super(
                // Ensure the elevator is clear before moving
                new ClearElevatorCommand(elevatorSubsystem).unless(() -> elevatorSubsystem.isClear()),
                // Parallel commands to put robot in face position
                Commands.parallel(
                        new RotateShoulderCommand(shoulderSubsystem, algaeRotationRotation),
                        new MoveElevatorCommand(elevatorSubsystem, algaeLevelSupplier)),
                // Turn on algae intake
                new IngestAlgaeCommand(algaeIntakeSubsystem));
    }
}
