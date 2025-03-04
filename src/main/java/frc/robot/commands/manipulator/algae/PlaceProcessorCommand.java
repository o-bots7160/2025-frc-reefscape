package frc.robot.commands.manipulator.algae;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TestLoggerCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class PlaceProcessorCommand extends SequentialCommandGroup {

    // Constructor
    public PlaceProcessorCommand(DriveBaseSubsystem drive, AlgaeIntakeSubsystem algae, ElevatorSubsystem elevator, ShoulderSubsystem shoulder,
            Pose2d faceTarget, Pose2d algaeTarget) {
        super(new TestLoggerCommand("Place Processor Direct"),
                Commands.parallel(drive.moveTo(faceTarget), elevator.goToCommand(0.0), shoulder.shoulderCommand(0.0)), drive.moveTo(algaeTarget));
    }
}
