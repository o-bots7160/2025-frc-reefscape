package frc.robot.config;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

@Logged
public class AllianceLandmarksConfig {
    public AllianceLandmarkConfig  blueAlliance;

    private AllianceLandmarkConfig redAlliance;

    public AllianceLandmarkConfig getBlueAlliance() {
        return blueAlliance;
    }

    public AllianceLandmarkConfig getRedAlliance() {
        // red alliance is a rotation of the blue alliance, adjust fields
        if (redAlliance == null) {
            redAlliance                   = new AllianceLandmarkConfig();
            redAlliance.joystickInversion = -blueAlliance.joystickInversion;
            redAlliance.startLeft         = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.startLeft);

            redAlliance.startMiddle       = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.startMiddle);

            redAlliance.startRight        = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.startRight);

            redAlliance.processor         = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.processor);

            redAlliance.coralStationLeft  = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.coralStationLeft);

            redAlliance.coralStationRight = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.coralStationRight);

            redAlliance.reefZoneA         = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneA);

            redAlliance.reefZoneB         = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneB);

            redAlliance.reefZoneC         = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneC);

            redAlliance.reefZoneD         = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneD);

            redAlliance.reefZoneE         = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneE);

            redAlliance.reefZoneF         = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneF);

            redAlliance.reefZoneG         = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneG);

            redAlliance.reefZoneH         = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneH);

            redAlliance.reefZoneI         = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneI);

            redAlliance.reefZoneJ         = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneJ);

            redAlliance.reefZoneK         = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneK);

            redAlliance.reefZoneL         = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneL);

            redAlliance.reefZoneAB        = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneAB);

            redAlliance.reefZoneCD        = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneCD);

            redAlliance.reefZoneEF        = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneEF);

            redAlliance.reefZoneGH        = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneGH);

            redAlliance.reefZoneIJ        = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneIJ);

            redAlliance.reefZoneKL        = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneKL);
        }

        return redAlliance;
    }

    public AllianceLandmarkConfig getAllianceLandmarkConfig(Alliance alliance) {
        return alliance == Alliance.Blue ? getBlueAlliance() : getRedAlliance();
    }

    /**
     * Shifts a given pose to the opposite alliance's coordinate frame by rotating it across the field center.
     *
     * @param fieldWidth  the width of the field
     * @param fieldLength the length of the field
     * @param pose        the original pose to shift
     * @return the shifted pose rotated across the center of the field
     */
    private Pose2d shiftPose(double fieldWidth, double fieldLength, Pose2d pose) {
        return new Pose2d(fieldLength - pose.getX(), fieldWidth - pose.getY(), pose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    }
}