package frc.robot.config;

import edu.wpi.first.math.util.Units;

public class DriveBaseSubsystemConfig {
    public double maximumSpeedInFeet;

    public double maximumSpeedInMeters() {
        return Units.feetToMeters(maximumSpeedInFeet);
    }
}