package frc.robot.config;

public abstract class AbstractIntakeSubsystemConfig extends AbstractSubsystemConfig {

    public double intakeSpeed;

    public double offDelay;

    public double onDelay;

    public double timeOfFlightSensorThreshold;

    public int    motorCanId;

    public int    timeOfFlightSensorCanId;
}
