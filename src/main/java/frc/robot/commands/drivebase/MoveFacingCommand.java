package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveBaseSubsystem;

//
// Command to move the robot manually while facing a pose on the field
//
//
public class MoveFacingCommand extends Command {
    /**
     *
     */
    private final DriveBaseSubsystem driveBaseSubsystem;

    private final DoubleSupplier     x;

    private final DoubleSupplier     y;

    private final Translation2d      targetTranslation;

    // Constructor
    public MoveFacingCommand(DriveBaseSubsystem subsystem, DoubleSupplier new_x, DoubleSupplier new_y,
            Translation2d new_translation) {
        super();
        driveBaseSubsystem = subsystem;
        x                  = new_x;
        y                  = new_y;
        targetTranslation  = new_translation;
        addRequirements(driveBaseSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        super.initialize();
        driveBaseSubsystem.setTarget(targetTranslation, this.driveBaseSubsystem.getPose().getTranslation());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        super.execute();
        var new_x = x.getAsDouble();
        var new_y = y.getAsDouble();
        driveBaseSubsystem.driveFacingTarget(new_x, new_y);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driveBaseSubsystem.stop();
    }
}