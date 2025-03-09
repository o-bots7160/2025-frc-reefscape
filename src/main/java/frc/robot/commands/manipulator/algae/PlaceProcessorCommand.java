package frc.robot.commands.manipulator.algae;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.MoveElevatorCommand;
import frc.robot.commands.manipulator.shoulder.RotateShoulderCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class PlaceProcessorCommand extends SequentialCommandGroup {

    // Constructor
    public PlaceProcessorCommand(DriveBaseSubsystem drive, AlgaeIntakeSubsystem algae, ElevatorSubsystem elevator, ShoulderSubsystem shoulder,
            Pose2d faceTarget, Pose2d algaeTarget) {
        super(Commands.parallel(drive.moveTo(faceTarget), new MoveElevatorCommand(elevator, 0.0), new RotateShoulderCommand(shoulder, 0.0)),
                drive.moveTo(algaeTarget));
    }
}
