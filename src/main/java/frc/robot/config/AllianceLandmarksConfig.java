package frc.robot.config;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

@Logged
public class AllianceLandmarksConfig {
    public AllianceLandmarkConfig blueAlliance;

    public AllianceLandmarkConfig redAlliance;

    public AllianceLandmarkConfig getAllianceLandmarkConfig(Alliance alliance) {
        return alliance == Alliance.Blue ? blueAlliance : redAlliance;
    }
}