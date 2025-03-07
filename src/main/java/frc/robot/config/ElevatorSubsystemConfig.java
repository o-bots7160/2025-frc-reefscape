package frc.robot.config;

import edu.wpi.first.epilogue.Logged;

@Logged
public class ElevatorSubsystemConfig extends SetAndSeekSubsystemConfigBase {

    public double clearedPosition;

    public double stowedPosition;

    public int    leftMotorCanId;

    public int    rightMotorCanId;
}
