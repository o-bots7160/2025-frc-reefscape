package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveBaseSubsystem;

/**
 * Command to stop the robot and put wheels in X formation
 */
public class StopCommand extends Command {
    private final DriveBaseSubsystem driveBaseSubsystem;

    public StopCommand(DriveBaseSubsystem driveBaseSubsystem) {
        this.driveBaseSubsystem = driveBaseSubsystem;
        addRequirements(driveBaseSubsystem);
    }

    @Override
    public void execute() {
        driveBaseSubsystem.lockSwerveDrivePose();
    }
}