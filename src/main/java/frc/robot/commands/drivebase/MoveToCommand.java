package frc.robot.commands.drivebase;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveBaseSubsystem;

/**
 * Command to move the robot to a new pose
 */
public class MoveToCommand extends Command {

    private final DriveBaseSubsystem driveBaseSubsystem;

    private final Supplier<Pose2d>   pose;

    public MoveToCommand(DriveBaseSubsystem driveBaseSubsystem, Pose2d pose) {
        this(driveBaseSubsystem, () -> pose);
    }

    public MoveToCommand(DriveBaseSubsystem subsystem, Supplier<Pose2d> pose) {
        driveBaseSubsystem = subsystem;
        this.pose          = pose;
        addRequirements(driveBaseSubsystem);
    }

    @Override
    public void initialize() {
        driveBaseSubsystem.setTarget(pose.get(), this.driveBaseSubsystem.getPose());
    }

    @Override
    public void execute() {
        driveBaseSubsystem.seekTarget();
    }

    @Override
    public void end(boolean interrupted) {
        driveBaseSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return driveBaseSubsystem.atTarget();
    }
}