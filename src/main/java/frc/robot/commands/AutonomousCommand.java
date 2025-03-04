package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveBaseSubsystem;

/**
 *
 */
@Logged
public class AutonomousCommand extends Command {

    private final DriveBaseSubsystem m_driveBaseSubsystem;

    public AutonomousCommand(DriveBaseSubsystem subsystem) {
        m_driveBaseSubsystem = subsystem;
        addRequirements(m_driveBaseSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
