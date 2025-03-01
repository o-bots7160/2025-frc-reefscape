package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceLandmarks {
    public Alliance current_alliance;

    public double   joystickInversion;

    public double   algaeHigh                 = 0.75;

    public double   algaeLow                  = 05;

    public double   coralLevel1               = 0.25;

    public double   coralLevel2               = 0.5;

    public double   coralLevel3               = 0.75;

    public double   coralLevel4               = 1.0;

    public double   fieldLength               = 17.548;

    public double   fieldWidth                = 8.052;

    public double   blueJoystickInversion     = 1.0;

    public Pose2d   blueStartLeft             = new Pose2d(7.241, 7.136, new Rotation2d(0.0));

    public Pose2d   blueStartMiddle           = new Pose2d(7.241, 3.887, new Rotation2d(0.0));

    public Pose2d   blueStartRight            = new Pose2d(7.241, 1.909, new Rotation2d(0.0));

    public Pose2d   blueProcessor             = new Pose2d(6.160, 0.625, new Rotation2d(180.0));

    public Pose2d   blueProcessorFace         = new Pose2d(6.160, 0.866, new Rotation2d(180.0));

    public Pose2d   blueCoralStationLeft      = new Pose2d(1.618, 7.158, new Rotation2d(-145.0));

    public Pose2d   blueCoralStationLeftFace  = new Pose2d(1.726, 7.100, new Rotation2d(-145.0));

    public Pose2d   blueCoralStationRight     = new Pose2d(1.618, 0.842, new Rotation2d(-35.0));

    public Pose2d   blueCoralStationRightFace = new Pose2d(1.726, 0.938, new Rotation2d(-35.0));

    public Pose2d   blueReefZoneA             = new Pose2d(3.093, 4.446, new Rotation2d(-90.0));

    public Pose2d   blueReefZoneB             = new Pose2d(3.093, 4.127, new Rotation2d(-90.0));

    public Pose2d   blueReefZoneC             = new Pose2d(3.492, 2.993, new Rotation2d(-30.0));

    public Pose2d   blueReefZoneD             = new Pose2d(3.747, 2.842, new Rotation2d(-30.0));

    public Pose2d   blueReefZoneE             = new Pose2d(4.867, 2.618, new Rotation2d(30.0));

    public Pose2d   blueReefZoneF             = new Pose2d(5.096, 2.777, new Rotation2d(30.0));

    public Pose2d   blueReefZoneG             = new Pose2d(5.874, 3.656, new Rotation2d(90.0));

    public Pose2d   blueReefZoneH             = new Pose2d(5.874, 3.934, new Rotation2d(90.0));

    public Pose2d   blueReefZoneI             = new Pose2d(5.473, 5.058, new Rotation2d(150.0));

    public Pose2d   blueReefZoneJ             = new Pose2d(5.207, 5.203, new Rotation2d(150.0));

    public Pose2d   blueReefZoneK             = new Pose2d(4.129, 5.418, new Rotation2d(-150.0));

    public Pose2d   blueReefZoneL             = new Pose2d(3.786, 5.260, new Rotation2d(-150.0));

    public Pose2d   blueReefZoneAB            = new Pose2d(3.093, 3.817, new Rotation2d(90.0));

    public Pose2d   blueReefZoneCD            = new Pose2d(3.948, 2.697, new Rotation2d(150.0));

    public Pose2d   blueReefZoneEF            = new Pose2d(5.348, 2.897, new Rotation2d(-150.0));

    public Pose2d   blueReefZoneGH            = new Pose2d(5.874, 4.254, new Rotation2d(-90.0));

    public Pose2d   blueReefZoneIJ            = new Pose2d(5.036, 5.357, new Rotation2d(-30.0));

    public Pose2d   blueReefZoneKL            = new Pose2d(3.546, 5.137, new Rotation2d(30.0));

    public Pose2d   blueReefFaceAB            = new Pose2d(2.901, 4.091, new Rotation2d(90.0));

    public Pose2d   blueReefFaceCD            = new Pose2d(3.620, 2.676, new Rotation2d(150.0));

    public Pose2d   blueReefFaceEF            = new Pose2d(5.275, 2.676, new Rotation2d(-150.0));

    public Pose2d   blueReefFaceGH            = new Pose2d(6.114, 4.091, new Rotation2d(-90.0));

    public Pose2d   blueReefFaceIJ            = new Pose2d(5.526, 5.266, new Rotation2d(-30.0));

    public Pose2d   blueReefFaceKL            = new Pose2d(3.488, 5.266, new Rotation2d(30.0));

    public double   redJoystickInversion      = -1.0;

    public Pose2d   redStartLeft              = new Pose2d(fieldLength - blueStartLeft.getX(),
            fieldWidth - blueStartLeft.getY(), new Rotation2d(180.0));

    public Pose2d   redStartMiddle            = new Pose2d(fieldLength - blueStartMiddle.getX(),
            fieldWidth - blueStartMiddle.getY(), new Rotation2d(180.0));

    public Pose2d   redStartRight             = new Pose2d(fieldLength - blueStartRight.getX(),
            fieldWidth - blueStartRight.getY(), new Rotation2d(180.0));

    public Pose2d   redProcessor              = new Pose2d(fieldLength - blueProcessor.getX(),
            fieldWidth - blueProcessor.getY(), new Rotation2d(0.0));

    public Pose2d   redProcessorFace          = new Pose2d(fieldLength - blueProcessorFace.getX(),
            fieldWidth - blueProcessorFace.getY(), new Rotation2d(0.0));

    public Pose2d   redCoralStationLeft       = new Pose2d(fieldLength - blueCoralStationLeft.getX(),
            fieldWidth - blueCoralStationLeft.getY(), new Rotation2d(35.0));

    public Pose2d   redCoralStationLeftFace   = new Pose2d(fieldLength - blueCoralStationLeftFace.getX(),
            fieldWidth - blueCoralStationLeftFace.getY(), new Rotation2d(35.0));

    public Pose2d   redCoralStationRight      = new Pose2d(fieldLength - blueCoralStationRight.getX(),
            fieldWidth - blueCoralStationRight.getY(), new Rotation2d(145.0));

    public Pose2d   redCoralStationRightFace  = new Pose2d(fieldLength - blueCoralStationRightFace.getX(),
            fieldWidth - blueCoralStationRightFace.getY(), new Rotation2d(145.0));

    public Pose2d   redReefZoneA              = new Pose2d(fieldLength - blueReefZoneA.getX(),
            fieldWidth - blueReefZoneA.getY(), new Rotation2d(90.0));

    public Pose2d   redReefZoneB              = new Pose2d(fieldLength - blueReefZoneB.getX(),
            fieldWidth - blueReefZoneB.getY(), new Rotation2d(90.0));

    public Pose2d   redReefZoneC              = new Pose2d(fieldLength - blueReefZoneC.getX(),
            fieldWidth - blueReefZoneC.getY(), new Rotation2d(150.0));

    public Pose2d   redReefZoneD              = new Pose2d(fieldLength - blueReefZoneD.getX(),
            fieldWidth - blueReefZoneD.getY(), new Rotation2d(150.0));

    public Pose2d   redReefZoneE              = new Pose2d(fieldLength - blueReefZoneE.getX(),
            fieldWidth - blueReefZoneE.getY(), new Rotation2d(-150.0));

    public Pose2d   redReefZoneF              = new Pose2d(fieldLength - blueReefZoneF.getX(),
            fieldWidth - blueReefZoneF.getY(), new Rotation2d(-150.0));

    public Pose2d   redReefZoneG              = new Pose2d(fieldLength - blueReefZoneG.getX(),
            fieldWidth - blueReefZoneG.getY(), new Rotation2d(-90.0));

    public Pose2d   redReefZoneH              = new Pose2d(fieldLength - blueReefZoneH.getX(),
            fieldWidth - blueReefZoneH.getY(), new Rotation2d(-90.0));

    public Pose2d   redReefZoneI              = new Pose2d(fieldLength - blueReefZoneI.getX(),
            fieldWidth - blueReefZoneI.getY(), new Rotation2d(-30.0));

    public Pose2d   redReefZoneJ              = new Pose2d(fieldLength - blueReefZoneJ.getX(),
            fieldWidth - blueReefZoneJ.getY(), new Rotation2d(-30.0));

    public Pose2d   redReefZoneK              = new Pose2d(fieldLength - blueReefZoneK.getX(),
            fieldWidth - blueReefZoneK.getY(), new Rotation2d(30.0));

    public Pose2d   redReefZoneL              = new Pose2d(fieldLength - blueReefZoneL.getX(),
            fieldWidth - blueReefZoneL.getY(), new Rotation2d(30.0));

    public Pose2d   redReefZoneAB             = new Pose2d(fieldLength - blueReefZoneAB.getX(),
            fieldWidth - blueReefZoneAB.getY(), new Rotation2d(-90.0));

    public Pose2d   redReefZoneCD             = new Pose2d(fieldLength - blueReefZoneCD.getX(),
            fieldWidth - blueReefZoneCD.getY(), new Rotation2d(-30.0));

    public Pose2d   redReefZoneEF             = new Pose2d(fieldLength - blueReefZoneEF.getX(),
            fieldWidth - blueReefZoneEF.getY(), new Rotation2d(30.0));

    public Pose2d   redReefZoneGH             = new Pose2d(fieldLength - blueReefZoneGH.getX(),
            fieldWidth - blueReefZoneGH.getY(), new Rotation2d(90.0));

    public Pose2d   redReefZoneIJ             = new Pose2d(fieldLength - blueReefZoneIJ.getX(),
            fieldWidth - blueReefZoneIJ.getY(), new Rotation2d(150.0));

    public Pose2d   redReefZoneKL             = new Pose2d(fieldLength - blueReefZoneKL.getX(),
            fieldWidth - blueReefZoneKL.getY(), new Rotation2d(-150.0));

    public Pose2d   redReefFaceAB             = new Pose2d(fieldLength - blueReefFaceAB.getX(),
            fieldWidth - blueReefFaceAB.getY(), new Rotation2d(-90.0));

    public Pose2d   redReefFaceCD             = new Pose2d(fieldLength - blueReefFaceCD.getX(),
            fieldWidth - blueReefFaceCD.getY(), new Rotation2d(-30.0));

    public Pose2d   redReefFaceEF             = new Pose2d(fieldLength - blueReefFaceEF.getX(),
            fieldWidth - blueReefFaceEF.getY(), new Rotation2d(30.0));

    public Pose2d   redReefFaceGH             = new Pose2d(fieldLength - blueReefFaceGH.getX(),
            fieldWidth - blueReefFaceGH.getY(), new Rotation2d(90.0));

    public Pose2d   redReefFaceIJ             = new Pose2d(fieldLength - blueReefFaceIJ.getX(),
            fieldWidth - blueReefFaceIJ.getY(), new Rotation2d(150.0));

    public Pose2d   redReefFaceKL             = new Pose2d(fieldLength - blueReefFaceKL.getX(),
            fieldWidth - blueReefFaceKL.getY(), new Rotation2d(-150.0));

    public Pose2d   startLeft                 = blueStartLeft;

    public Pose2d   startMiddle               = blueStartMiddle;

    public Pose2d   startRight                = blueStartRight;

    public Pose2d   processor                 = blueProcessor;

    public Pose2d   processorFace             = blueProcessorFace;

    public Pose2d   coralStationLeft          = blueCoralStationLeft;

    public Pose2d   coralStationLeftFace      = blueCoralStationLeftFace;

    public Pose2d   coralStationRight         = blueCoralStationRight;

    public Pose2d   coralStationRightFace     = blueCoralStationRightFace;

    public Pose2d   reefZoneA                 = blueReefZoneA;

    public Pose2d   reefZoneB                 = blueReefZoneB;

    public Pose2d   reefZoneC                 = blueReefZoneC;

    public Pose2d   reefZoneD                 = blueReefZoneD;

    public Pose2d   reefZoneE                 = blueReefZoneE;

    public Pose2d   reefZoneF                 = blueReefZoneF;

    public Pose2d   reefZoneG                 = blueReefZoneG;

    public Pose2d   reefZoneH                 = blueReefZoneH;

    public Pose2d   reefZoneI                 = blueReefZoneI;

    public Pose2d   reefZoneJ                 = blueReefZoneJ;

    public Pose2d   reefZoneK                 = blueReefZoneK;

    public Pose2d   reefZoneL                 = blueReefZoneL;

    public Pose2d   reefZoneAB                = blueReefZoneAB;

    public Pose2d   reefZoneCD                = blueReefZoneCD;

    public Pose2d   reefZoneEF                = blueReefZoneEF;

    public Pose2d   reefZoneGH                = blueReefZoneGH;

    public Pose2d   reefZoneIJ                = blueReefZoneIJ;

    public Pose2d   reefZoneKL                = blueReefZoneKL;

    public Pose2d   reefFaceAB                = blueReefFaceAB;

    public Pose2d   reefFaceCD                = blueReefFaceCD;

    public Pose2d   reefFaceEF                = blueReefFaceEF;

    public Pose2d   reefFaceGH                = blueReefFaceGH;

    public Pose2d   reefFaceIJ                = blueReefFaceIJ;

    public Pose2d   reefFaceKL                = blueReefFaceKL;

    public void newAlliance(Alliance alliance) {
        current_alliance = alliance;
        if (alliance == Alliance.Blue) {
            joystickInversion     = blueJoystickInversion;
            startLeft             = blueStartLeft;
            startMiddle           = blueStartMiddle;
            startRight            = blueStartRight;
            processor             = blueProcessor;
            processorFace         = blueProcessorFace;
            coralStationLeft      = blueCoralStationLeft;
            coralStationLeftFace  = blueCoralStationLeftFace;
            coralStationRight     = blueCoralStationRight;
            coralStationRightFace = blueCoralStationRightFace;
            reefZoneA             = blueReefZoneA;
            reefZoneB             = blueReefZoneB;
            reefZoneC             = blueReefZoneC;
            reefZoneD             = blueReefZoneD;
            reefZoneE             = blueReefZoneE;
            reefZoneF             = blueReefZoneF;
            reefZoneG             = blueReefZoneG;
            reefZoneH             = blueReefZoneH;
            reefZoneI             = blueReefZoneI;
            reefZoneJ             = blueReefZoneJ;
            reefZoneK             = blueReefZoneK;
            reefZoneL             = blueReefZoneL;
            reefZoneAB            = blueReefZoneAB;
            reefZoneCD            = blueReefZoneCD;
            reefZoneEF            = blueReefZoneEF;
            reefZoneGH            = blueReefZoneGH;
            reefZoneIJ            = blueReefZoneIJ;
            reefZoneKL            = blueReefZoneKL;
            reefFaceAB            = blueReefFaceAB;
            reefFaceCD            = blueReefFaceCD;
            reefFaceEF            = blueReefFaceEF;
            reefFaceGH            = blueReefFaceGH;
            reefFaceIJ            = blueReefFaceIJ;
            reefFaceKL            = blueReefFaceKL;
        } else {
            joystickInversion     = redJoystickInversion;
            startLeft             = redStartLeft;
            startMiddle           = redStartMiddle;
            startRight            = redStartRight;
            processor             = redProcessor;
            processorFace         = redProcessorFace;
            coralStationLeft      = redCoralStationLeft;
            coralStationLeftFace  = redCoralStationLeftFace;
            coralStationRight     = redCoralStationRight;
            coralStationRightFace = redCoralStationRightFace;
            reefZoneA             = redReefZoneA;
            reefZoneB             = redReefZoneB;
            reefZoneC             = redReefZoneC;
            reefZoneD             = redReefZoneD;
            reefZoneE             = redReefZoneE;
            reefZoneF             = redReefZoneF;
            reefZoneG             = redReefZoneG;
            reefZoneH             = redReefZoneH;
            reefZoneI             = redReefZoneI;
            reefZoneJ             = redReefZoneJ;
            reefZoneK             = redReefZoneK;
            reefZoneL             = redReefZoneL;
            reefZoneAB            = redReefZoneAB;
            reefZoneCD            = redReefZoneCD;
            reefZoneEF            = redReefZoneEF;
            reefZoneGH            = redReefZoneGH;
            reefZoneIJ            = redReefZoneIJ;
            reefZoneKL            = redReefZoneKL;
            reefFaceAB            = redReefFaceAB;
            reefFaceCD            = redReefFaceCD;
            reefFaceEF            = redReefFaceEF;
            reefFaceGH            = redReefFaceGH;
            reefFaceIJ            = redReefFaceIJ;
            reefFaceKL            = redReefFaceKL;
        }
    }
}