package frc.robot.commands.drivebase;

import frc.robot.subsystems.DriveBaseSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ResetAngleCommand extends Command{
    
    private DriveBaseSubsystem driveBaseSubsystem;

    private double angle;

    public ResetAngleCommand(double angle, DriveBaseSubsystem driveBaseSubsystem) {
        this.angle = angle;
        this.driveBaseSubsystem = driveBaseSubsystem;
    }

    @Override
    public void initialize() {
        driveBaseSubsystem.resetAngle(angle);
    }
}
