package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveBaseSubsystem;

/**
 * Command to manually move the robot using joysticks field oriented
 */
public class MoveManualCommandField extends Command {
    private final DoubleSupplier     x;

    private final DoubleSupplier     y;

    private final DoubleSupplier     r;

    private final DriveBaseSubsystem driveBaseSubsystem;

    public MoveManualCommandField(DriveBaseSubsystem driveBaseSubsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
        this.driveBaseSubsystem = driveBaseSubsystem;
        this.x                  = x;
        this.y                  = y;
        this.r                  = rotation;
        addRequirements(driveBaseSubsystem);
    }

    @Override
    public void execute() {
        driveBaseSubsystem.driveField(x.getAsDouble(), y.getAsDouble(), r.getAsDouble() * -1.0);
    }

    @Override
    public void end(boolean interrupted) {
        driveBaseSubsystem.stop();
    }
}