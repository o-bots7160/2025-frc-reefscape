package frc.robot.config;

public abstract class IntakeSubsystemConfigBase extends SubsystemConfigBase {
    public int    motorCanId;

    public int    timeOfFlightSensorCanId;

    public double timeOfFlightSensorThreshold;

    public double onDelay;

    public double offDelay;
}
