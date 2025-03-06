package frc.robot.config;

import edu.wpi.first.epilogue.Logged;

@Logged
public class ElevatorSubsystemConfig extends SubsystemConfigBase {

    public double clearHeight;

    public double maximumHeight;

    public double minimumHeight;

    public double stowHeight;

    public int    leftMotorCanId;

    public int    rightMotorCanId;
}
