package frc.robot.config;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;

@Logged
public class DriveBaseSubsystemConfig {
    public double maximumSpeedInFeet;
    public double thetaControllerTolerance;
    public double thetaControllerPidKp;
    public double thetaControllerPidKi;
    public double thetaControllerPidKd;

    public double maximumSpeedInMeters() {
        return Units.feetToMeters(maximumSpeedInFeet);
    }
}