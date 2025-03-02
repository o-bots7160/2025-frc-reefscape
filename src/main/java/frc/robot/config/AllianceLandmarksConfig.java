package frc.robot.config;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

@Logged
public class AllianceLandmarksConfig {
    public AllianceLandmarkConfig  referenceAlliance;

    private AllianceLandmarkConfig redAlliance;

    public AllianceLandmarkConfig getBlueAlliance() {
        return referenceAlliance;
    }

    public AllianceLandmarkConfig getRedAlliance() {
        // red alliance is a rotation of the blue alliance, adjust fields
        if (redAlliance == null) {
            redAlliance                   = new AllianceLandmarkConfig();
            redAlliance.joystickInversion = -referenceAlliance.joystickInversion;
            redAlliance.startLeft         = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.startLeft);

            redAlliance.startMiddle       = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.startMiddle);

            redAlliance.startRight        = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.startRight);

            redAlliance.processor         = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.processor);

            redAlliance.coralStationLeft  = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.coralStationLeft);

            redAlliance.coralStationRight = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.coralStationRight);

            redAlliance.reefZoneA         = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.reefZoneA);

            redAlliance.reefZoneB         = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.reefZoneB);

            redAlliance.reefZoneC         = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.reefZoneC);

            redAlliance.reefZoneD         = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.reefZoneD);

            redAlliance.reefZoneE         = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.reefZoneE);

            redAlliance.reefZoneF         = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.reefZoneF);

            redAlliance.reefZoneG         = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.reefZoneG);

            redAlliance.reefZoneH         = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.reefZoneH);

            redAlliance.reefZoneI         = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.reefZoneI);

            redAlliance.reefZoneJ         = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.reefZoneJ);

            redAlliance.reefZoneK         = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.reefZoneK);

            redAlliance.reefZoneL         = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.reefZoneL);

            redAlliance.reefZoneAB        = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.reefZoneAB);

            redAlliance.reefZoneCD        = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.reefZoneCD);

            redAlliance.reefZoneEF        = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.reefZoneEF);

            redAlliance.reefZoneGH        = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.reefZoneGH);

            redAlliance.reefZoneIJ        = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.reefZoneIJ);

            redAlliance.reefZoneKL        = shiftPose(referenceAlliance.fieldWidth, referenceAlliance.fieldLength,
                    referenceAlliance.reefZoneKL);
        }

        return redAlliance;
    }

    public AllianceLandmarkConfig getAllianceLandmarkConfig(Alliance alliance) {
        return alliance == Alliance.Blue ? getBlueAlliance() : getRedAlliance();
    }

    /**
     * Shifts a given pose to the opposite alliance's coordinate frame by rotating
     * it across the field center.
     *
     * @param fieldWidth  the width of the field
     * @param fieldLength the length of the field
     * @param pose        the original pose to shift
     * @return the shifted pose rotated across the center of the field
     */
    private Pose2d shiftPose(double fieldWidth, double fieldLength, Pose2d pose) {
        return new Pose2d(fieldLength - pose.getX(), fieldWidth - pose.getY(),
                pose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    }
}