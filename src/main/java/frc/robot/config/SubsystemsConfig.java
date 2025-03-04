package frc.robot.config;

import edu.wpi.first.epilogue.Logged;

@Logged
public class SubsystemsConfig {
    public DriveBaseSubsystemConfig   driveBaseSubsystem;

    public AlgaeIntakeSubsystemConfig algaeIntakeSubsystem;

    public CoralIntakeSubsystemConfig coralIntakeSubsystem;

    public ClimberSubsystemConfig     climberSubsystem;

    public ElevatorSubsystemConfig    elevatorSubsystem;

    public ShoulderSubsystemConfig    shoulderSubsystem;

    public boolean                    verboseOutput;
}
