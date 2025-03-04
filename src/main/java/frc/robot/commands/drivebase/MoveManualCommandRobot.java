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

    private final DoubleSupplier     r;

    private final DriveBaseSubsystem driveBase;

    // Constructor
    public MoveManualCommandRobot(DriveBaseSubsystem subsystem, DoubleSupplier new_x, DoubleSupplier new_y, DoubleSupplier new_r) {
        super();
        x         = new_x;
        y         = new_y;
        r         = new_r;
        driveBase = subsystem;
        addRequirements(driveBase);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        super.execute();
        var new_x = x.getAsDouble();
        var new_y = y.getAsDouble();
        var new_r = -r.getAsDouble();
        driveBase.driveRobot(new_x, new_y, new_r);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driveBase.stop();
    }
}