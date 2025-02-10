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

    private final Rotation2d         r;

    // Constructor
    public MoveAtAngle(DriveBaseSubsystem subsystem, DoubleSupplier new_x, DoubleSupplier new_y, Rotation2d new_r) {
        super();
        driveBaseSubsystem = subsystem;
        x                  = new_x;
        y                  = new_y;
        r                  = new_r;
        addRequirements(driveBaseSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        super.initialize();
        driveBaseSubsystem.setTarget(r, this.driveBaseSubsystem.getPose().getRotation());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        super.execute();
        var new_x = x.getAsDouble();
        var new_y = y.getAsDouble();
        driveBaseSubsystem.driveAtAngle(new_x, new_y);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driveBaseSubsystem.stop();
    }
}