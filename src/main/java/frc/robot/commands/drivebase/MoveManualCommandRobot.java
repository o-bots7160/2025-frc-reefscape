package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveBaseSubsystem;

//
// Command to manually move the robot using joysticks field oriented
//
//
public class MoveManualCommandRobot extends Command {
    private final DoubleSupplier     x;

    private final DoubleSupplier     y;

    private final DoubleSupplier     rotation;

    private final DriveBaseSubsystem driveBaseSubsystem;

    public MoveManualCommandRobot(DriveBaseSubsystem driveBaseSubsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
        this.driveBaseSubsystem = driveBaseSubsystem;
        this.x                  = x;
        this.y                  = y;
        this.rotation           = rotation;
        addRequirements(driveBaseSubsystem);
    }

    @Override
    public void execute() {
        driveBaseSubsystem.driveRobot(x.getAsDouble(), y.getAsDouble(), rotation.getAsDouble() * -1.0);
    }

    @Override
    public void end(boolean interrupted) {
        driveBaseSubsystem.stop();
    }
}