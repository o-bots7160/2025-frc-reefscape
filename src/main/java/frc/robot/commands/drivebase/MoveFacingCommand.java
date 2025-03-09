package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveBaseSubsystem;

/**
 * Command to move the robot manually while facing a pose on the field
 */
public class MoveFacingCommand extends Command {

    private final DriveBaseSubsystem driveBaseSubsystem;

    private final DoubleSupplier     x;

    private final DoubleSupplier     y;

    private final Translation2d      translation;

    public MoveFacingCommand(DriveBaseSubsystem driveBaseSubsystem, DoubleSupplier x, DoubleSupplier y, Translation2d translation) {
        super();
        this.driveBaseSubsystem = driveBaseSubsystem;
        this.x                  = x;
        this.y                  = y;
        this.translation        = translation;
        addRequirements(driveBaseSubsystem);
    }

    @Override
    public void initialize() {
        driveBaseSubsystem.setTarget(translation, driveBaseSubsystem.getPose().getTranslation());
    }

    @Override
    public void execute() {
        driveBaseSubsystem.driveFacingTarget(x.getAsDouble(), y.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveBaseSubsystem.stop();
    }
}