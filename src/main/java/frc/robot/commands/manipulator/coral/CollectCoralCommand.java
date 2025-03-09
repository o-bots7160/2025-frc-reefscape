// package frc.robot.commands.manipulator.coral;

// import java.util.function.Supplier;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.elevator.MoveElevatorCommand;
// import frc.robot.commands.manipulator.RotateShoulderCommand;
// import frc.robot.subsystems.CoralIntakeSubsystem;
// import frc.robot.subsystems.DriveBaseSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.ShoulderSubsystem;

// public class CollectCoralCommand extends SequentialCommandGroup {

//     // Constructor
//     public CollectCoralCommand(DriveBaseSubsystem drive, CoralIntakeSubsystem coral, ElevatorSubsystem elevator, ShoulderSubsystem shoulder,
//             Pose2d faceTarget, Pose2d coralTarget) {
//         super(Commands.parallel(drive.moveTo(faceTarget), new MoveElevatorCommand(elevator, 0.0), new RotateShoulderCommand(shoulder, 0.0)),
//                 drive.moveTo(coralTarget),
//                 coral.inject(),
//                 Commands.parallel(drive.moveTo(faceTarget), new MoveElevatorCommand(elevator, 0.0), new RotateShoulderCommand(shoulder, 0.0)));
//     }

//     // Constructor
//     public CollectCoralCommand(DriveBaseSubsystem drive, CoralIntakeSubsystem coral, ElevatorSubsystem elevator, ShoulderSubsystem shoulder,
//             Supplier<Pose2d> faceTarget, Supplier<Pose2d> coralTarget) {
//         super(Commands.parallel(drive.moveTo(faceTarget.get()), new MoveElevatorCommand(elevator, 0.0), new RotateShoulderCommand(shoulder, 0.0)),
//                 drive.moveTo(coralTarget.get()), coral.inject(),
//                 Commands.parallel(drive.moveTo(faceTarget), new MoveElevatorCommand(elevator, 0.0), new RotateShoulderCommand(shoulder, 0.0)));
//     }
// }
