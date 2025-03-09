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
            redAlliance                       = new AllianceLandmarkConfig();

            redAlliance.algaeHigh             = blueAlliance.algaeHigh;
            redAlliance.algaeHighRotation     = blueAlliance.algaeHighRotation;
            redAlliance.algaeLow              = blueAlliance.algaeLow;
            redAlliance.algaeLowRotation      = blueAlliance.algaeLowRotation;
            redAlliance.coralStationHeight    = blueAlliance.coralStationHeight;
            redAlliance.coralStationRotation  = blueAlliance.coralStationRotation;
            redAlliance.coralLevel1           = blueAlliance.coralLevel1;
            redAlliance.coralLevel1Rotation   = blueAlliance.coralLevel1Rotation;
            redAlliance.coralLevel2           = blueAlliance.coralLevel2;
            redAlliance.coralLevel2Rotation   = blueAlliance.coralLevel2Rotation;
            redAlliance.coralLevel3           = blueAlliance.coralLevel3;
            redAlliance.coralLevel3Rotation   = blueAlliance.coralLevel3Rotation;
            redAlliance.coralLevel4           = blueAlliance.coralLevel4;
            redAlliance.coralLevel4Rotation   = blueAlliance.coralLevel4Rotation;
            redAlliance.fieldLength           = blueAlliance.fieldLength;
            redAlliance.fieldWidth            = blueAlliance.fieldWidth;

            // Override blue alliance locations where the rotation effects them

            redAlliance.coralStationLeft      = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.coralStationLeft);
            redAlliance.coralStationLeftFace  = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.coralStationLeftFace);
            redAlliance.coralStationRight     = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.coralStationRight);
            redAlliance.coralStationRightFace = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.coralStationRightFace);
            redAlliance.joystickInversion     = -blueAlliance.joystickInversion;
            redAlliance.processor             = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.processor);
            redAlliance.processorFace         = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.processorFace);
            redAlliance.reefZoneA             = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneA);
            redAlliance.reefZoneAB            = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneAB);
            redAlliance.reefZoneB             = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneB);
            redAlliance.reefZoneC             = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneC);
            redAlliance.reefZoneCD            = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneCD);
            redAlliance.reefZoneD             = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneD);
            redAlliance.reefZoneE             = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneE);
            redAlliance.reefZoneEF            = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneEF);
            redAlliance.reefZoneF             = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneF);
            redAlliance.reefZoneG             = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneG);
            redAlliance.reefZoneGH            = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneGH);
            redAlliance.reefZoneH             = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneH);
            redAlliance.reefZoneI             = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneI);
            redAlliance.reefZoneIJ            = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneIJ);
            redAlliance.reefZoneJ             = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneJ);
            redAlliance.reefZoneK             = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneK);
            redAlliance.reefZoneKL            = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneKL);
            redAlliance.reefZoneL             = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.reefZoneL);
            redAlliance.startLeft             = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.startLeft);
            redAlliance.startMiddle           = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.startMiddle);
            redAlliance.startRight            = shiftPose(blueAlliance.fieldWidth, blueAlliance.fieldLength, blueAlliance.startRight);
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