package frc.robot.commands.drivebase;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveBaseSubsystem;

//
// Command to move the robot to a new pose
//
//
public class MoveToCommand extends Command {
    /**
     *
     */
    private final DriveBaseSubsystem driveBaseSubsystem;

    private       Pose2d           targetPose;
    private final Supplier<Pose2d> supplier;

    // Constructor
    public MoveToCommand(DriveBaseSubsystem subsystem, Pose2d new_pose) {
        super();
        driveBaseSubsystem = subsystem;
        targetPose         = new_pose;
        supplier           = null;
        addRequirements(driveBaseSubsystem);
    }

    // Constructor
    public MoveToCommand(DriveBaseSubsystem subsystem, Supplier<Pose2d> new_pose) {
        super();
        driveBaseSubsystem = subsystem;
        targetPose         = new Pose2d();
        supplier           = new_pose;
        addRequirements(driveBaseSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        super.initialize();
        if ( supplier != null ) {
            targetPose = supplier.get();
        }
        driveBaseSubsystem.setTarget(targetPose, this.driveBaseSubsystem.getPose());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        super.execute();
        driveBaseSubsystem.driveToTarget();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driveBaseSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        super.isFinished();
        return driveBaseSubsystem.getHasTarget();
    }
}