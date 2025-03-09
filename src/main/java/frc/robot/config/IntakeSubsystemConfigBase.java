package frc.robot.config;

import edu.wpi.first.epilogue.Logged;

@Logged
public abstract class IntakeSubsystemConfigBase extends SubsystemConfigBase {

    public double intakeSpeed;

    public double offDelay;

    public double onDelay;

    public double timeOfFlightSensorThreshold;

    public int    motorCanId;

    public int    timeOfFlightSensorCanId;
}
