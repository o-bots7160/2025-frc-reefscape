package frc.robot.commands.manipulator.algae;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TestLoggerCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class NetCommand extends SequentialCommandGroup {

    // Constructor
    public NetCommand(AlgaeIntakeSubsystem algae, ElevatorSubsystem elevator, ShoulderSubsystem shoulder) {
        super(new TestLoggerCommand("Place Algae in Net"), Commands.parallel(elevator.goToCommand(0.0), shoulder.shoulderCommand(0.0)));
    }
}