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

    public double  kP;                  // position PID P

    public double  kI;                  // position PID I

    public double  kD;                  // position PID D

    public double  kS;                  // static feedforward

    public double  kV;                  // velocity feedforward

    public double  kA;                  // acceleration feedforward

    public double  kG;                  // constant gravity / bias term (if needed)
}
