package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveBaseSubsystem;
import swervelib.SwerveDriveTest;

public class DriveAngleSetCommand extends Command
{

    private final DriveBaseSubsystem m_driveBaseSubsystem;
    private Rotation2d m_angle;

    public DriveAngleSetCommand(Rotation2d angle, DriveBaseSubsystem subsystem) {

        m_driveBaseSubsystem = subsystem;
        m_angle              = angle;
        addRequirements(m_driveBaseSubsystem);

    }

    @Override
    public void execute() 
    {
        // TODO Auto-generated method stub
        super.execute();
        SwerveDriveTest.angleModules(m_driveBaseSubsystem.swerveDrive, m_angle);
    }

    @Override
    public void initialize() 
    {
        // TODO Auto-generated method stub
        super.initialize();
        System.out.println("initialize");
    }

    @Override
    public void end(boolean interrupted) 
    {
        System.out.println("end");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() 
    {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() 
    {
        return false;
    }
}
