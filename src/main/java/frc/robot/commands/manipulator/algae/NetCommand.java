package frc.robot.commands.manipulator.algae;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.MoveElevatorCommand;
import frc.robot.commands.manipulator.shoulder.RotateShoulderCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class NetCommand extends SequentialCommandGroup {

    // Constructor
    public NetCommand(AlgaeIntakeSubsystem algae, ElevatorSubsystem elevator, ShoulderSubsystem shoulder) {
        super(Commands.parallel(new MoveElevatorCommand(elevator, 0.0), new RotateShoulderCommand(shoulder, 0.0)));
    }
}