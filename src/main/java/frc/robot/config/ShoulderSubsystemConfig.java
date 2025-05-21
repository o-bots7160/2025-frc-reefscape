package frc.robot.config;

import edu.wpi.first.epilogue.Logged;

@Logged
public class ShoulderSubsystemConfig extends AbstractSetAndSeekSubsystemConfig {
    public int    motorCanId;

    public double rotationOffset;

    public double conversionFactor;
}
