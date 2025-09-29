package frc.robot.config;

public abstract class AbstractSetAndSeekSubsystemConfig extends AbstractSubsystemConfig {

    public boolean enableDefaultCommand;

    public double  clearedPosition;

    public double  maximumAcceleration;

    public double  maximumSetPoint;

    public double  maximumVelocity;

    public double  minimumSetPoint;

    public double  setPointTolerance;

    public double  stoppingTolerance;

    public double  stowedPosition;
}
