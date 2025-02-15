package frc.robot.config;

import edu.wpi.first.epilogue.Logged;

@Logged
public class SubsystemsConfig {
    public DriveBaseSubsystemConfig   driveBaseSubsystem;

    public AlgaeIntakeSubsystemConfig algaeIntakeSubsystem;

    public CoralIntakeSubsystemConfig coralIntakeSubsystem;

    public boolean                    verboseOutput;
}
