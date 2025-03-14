package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveBaseSubsystem;

public class ResetAngleCommand extends Command{
    
    private DriveBaseSubsystem driveBaseSubsystem;

    private double angle;

    public ResetAngleCommand(DriveBaseSubsystem driveBaseSubsystem, double angle) {
        this.driveBaseSubsystem = driveBaseSubsystem;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        driveBaseSubsystem.resetAngle(angle);
    }
}
