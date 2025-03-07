package frc.robot.config;

import edu.wpi.first.epilogue.Logged;

@Logged
public class ElevatorSubsystemConfig extends SetAndSeekSubsystemConfigBase {

    public double clearHeight;

    public double stowHeight;

    public int    leftMotorCanId;

    public int    rightMotorCanId;
}
