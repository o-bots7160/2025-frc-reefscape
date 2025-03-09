package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.manipulator.shoulder.RotateShoulderCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class TravelCommand extends SequentialCommandGroup {

    // Constructor
    public TravelCommand(ElevatorSubsystem elevatorSubsystem, ShoulderSubsystem shoulderSubsystem, Command clearElevatorCommand) {
        super(Commands.parallel(clearElevatorCommand, new RotateShoulderCommand(shoulderSubsystem, 0.0)));
    }
}