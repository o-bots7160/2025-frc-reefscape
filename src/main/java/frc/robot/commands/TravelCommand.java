package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class TravelCommand extends SequentialCommandGroup {

    // Constructor
    public TravelCommand(ElevatorSubsystem elevator, ShoulderSubsystem shoulder, Command clearElevatorCommand) {
        super(new TestLoggerCommand("Travel"), Commands.parallel(clearElevatorCommand, shoulder.shoulderCommand(0.0)));
    }
}