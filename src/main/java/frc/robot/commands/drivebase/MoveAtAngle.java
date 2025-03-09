package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveBaseSubsystem;

//
// Command to manually translate the robot using joystick while it faces an
// angle
//
//
public class MoveAtAngle extends Command {
    /**
     *
     */
    private final DriveBaseSubsystem driveBaseSubsystem;

    private final DoubleSupplier     x;

    private final DoubleSupplier     y;

    private final Rotation2d         rotation;

    // Constructor
    public MoveAtAngle(DriveBaseSubsystem driveBaseSubsystem, DoubleSupplier x, DoubleSupplier y, Rotation2d rotation) {
        this.driveBaseSubsystem = driveBaseSubsystem;
        this.x                  = x;
        this.y                  = y;
        this.rotation           = rotation;
        addRequirements(driveBaseSubsystem);
    }

    @Override
    public void initialize() {
        driveBaseSubsystem.setTarget(rotation, driveBaseSubsystem.getPose().getRotation());
    }

    @Override
    public void execute() {
        var new_x = x.getAsDouble();
        var new_y = y.getAsDouble();
        driveBaseSubsystem.driveAtAngle(new_x, new_y);
    }

    @Override
    public void end(boolean interrupted) {
        driveBaseSubsystem.stop();
    }
}