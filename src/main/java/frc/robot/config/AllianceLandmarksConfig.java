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
        blueAlliance.name = "Blue";
        return blueAlliance;
    }

    public AllianceLandmarkConfig getRedAlliance() {
        // red alliance is a rotation of the blue alliance, adjust fields
        if (redAlliance == null) {
            redAlliance                       = new AllianceLandmarkConfig();
            redAlliance.name                  = "Red";
            redAlliance.algaeHigh             = blueAlliance.algaeHigh;
            redAlliance.algaeHighRotation     = blueAlliance.algaeHighRotation;
            redAlliance.algaeLow              = blueAlliance.algaeLow;
            redAlliance.algaeLowRotation      = blueAlliance.algaeLowRotation;
            redAlliance.netHeight             = blueAlliance.netHeight;
            redAlliance.netRotation           = blueAlliance.netRotation;
            redAlliance.processorHeight       = blueAlliance.processorHeight;
            redAlliance.processorRotation     = blueAlliance.processorRotation;
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

            redAlliance.coralStationLeft      = shiftPose(blueAlliance.coralStationLeft);
            redAlliance.coralStationLeftFace  = shiftPose(blueAlliance.coralStationLeftFace);
            redAlliance.coralStationRight     = shiftPose(blueAlliance.coralStationRight);
            redAlliance.coralStationRightFace = shiftPose(blueAlliance.coralStationRightFace);
            redAlliance.joystickInversion     = -blueAlliance.joystickInversion;
            redAlliance.net                   = shiftPose(blueAlliance.net);
            redAlliance.processor             = shiftPose(blueAlliance.processor);
            redAlliance.processorFace         = shiftPose(blueAlliance.processorFace);
            redAlliance.reefZoneA             = shiftPose(blueAlliance.reefZoneA);
            redAlliance.reefZoneAB            = shiftPose(blueAlliance.reefZoneAB);
            redAlliance.reefZoneB             = shiftPose(blueAlliance.reefZoneB);
            redAlliance.reefZoneC             = shiftPose(blueAlliance.reefZoneC);
            redAlliance.reefZoneCD            = shiftPose(blueAlliance.reefZoneCD);
            redAlliance.reefZoneD             = shiftPose(blueAlliance.reefZoneD);
            redAlliance.reefZoneE             = shiftPose(blueAlliance.reefZoneE);
            redAlliance.reefZoneEF            = shiftPose(blueAlliance.reefZoneEF);
            redAlliance.reefZoneF             = shiftPose(blueAlliance.reefZoneF);
            redAlliance.reefZoneG             = shiftPose(blueAlliance.reefZoneG);
            redAlliance.reefZoneGH            = shiftPose(blueAlliance.reefZoneGH);
            redAlliance.reefZoneH             = shiftPose(blueAlliance.reefZoneH);
            redAlliance.reefZoneI             = shiftPose(blueAlliance.reefZoneI);
            redAlliance.reefZoneIJ            = shiftPose(blueAlliance.reefZoneIJ);
            redAlliance.reefZoneJ             = shiftPose(blueAlliance.reefZoneJ);
            redAlliance.reefZoneK             = shiftPose(blueAlliance.reefZoneK);
            redAlliance.reefZoneKL            = shiftPose(blueAlliance.reefZoneKL);
            redAlliance.reefZoneL             = shiftPose(blueAlliance.reefZoneL);
            redAlliance.reefFaceAB            = shiftPose(blueAlliance.reefFaceAB);
            redAlliance.reefFaceCD            = shiftPose(blueAlliance.reefFaceCD);
            redAlliance.reefFaceEF            = shiftPose(blueAlliance.reefFaceEF);
            redAlliance.reefFaceGH            = shiftPose(blueAlliance.reefFaceGH);
            redAlliance.reefFaceIJ            = shiftPose(blueAlliance.reefFaceIJ);
            redAlliance.reefFaceKL            = shiftPose(blueAlliance.reefFaceKL);
            redAlliance.startLeft             = shiftPose(blueAlliance.startLeft);
            redAlliance.startMiddle           = shiftPose(blueAlliance.startMiddle);
            redAlliance.startRight            = shiftPose(blueAlliance.startRight);
        }

        return redAlliance;
    }

    public AllianceLandmarkConfig getAllianceLandmarkConfig(Alliance alliance) {
        return alliance == Alliance.Blue ? getBlueAlliance() : getRedAlliance();
    }

    /**
     * Shifts a given pose to the opposite alliance's coordinate frame by rotating it across the field center.
     * 
     * @param pose the original pose to shift
     * @return the shifted pose rotated across the center of the field
     */
    private Pose2d shiftPose(Pose2d pose) {
        double fieldWidth  = blueAlliance.fieldWidth;
        double fieldLength = blueAlliance.fieldLength;
        return new Pose2d(fieldLength - pose.getX(), fieldWidth - pose.getY() - 0.09, pose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    }
}