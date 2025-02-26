package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceLandmarks {
    private double  blueJoystickInversion     = 1.0;
    private Pose2d  blueStartLeft             = new Pose2d(7.241, 7.136, new Rotation2d(0.0));
    private Pose2d  blueStartMiddle           = new Pose2d(7.241, 3.887, new Rotation2d(0.0));
    private Pose2d  blueStartRight            = new Pose2d(7.241, 1.909, new Rotation2d(0.0));
    private Pose2d  blueProcessor             = new Pose2d(6.066, 0.698, new Rotation2d(180.0));
    private Pose2d  blueProcessorFace         = new Pose2d(6.066, 0.698, new Rotation2d(180.0));  //FIXME
    private Pose2d  blueCoralStationLeft      = new Pose2d(1.618, 7.158, new Rotation2d(-35.0));
    private Pose2d  blueCoralStationLeftFace  = new Pose2d(1.618, 7.158, new Rotation2d(-35.0));        //FIXME
    private Pose2d  blueCoralStationRight     = new Pose2d(1.618, 0.842, new Rotation2d(-145.0)); 
    private Pose2d  blueCoralStationRightFace = new Pose2d(1.618, 0.842, new Rotation2d(-145.0));       //FIXME

    private Pose2d  blueReefZoneA         = new Pose2d(3.093, 4.223, new Rotation2d(-90.0));
    private Pose2d  blueReefZoneB         = new Pose2d(3.093, 3.851, new Rotation2d(-90.0));
    private Pose2d  blueReefZoneC         = new Pose2d(3.644, 2.940, new Rotation2d(-30.0));
    private Pose2d  blueReefZoneD         = new Pose2d(3.968, 2.760, new Rotation2d(-30.0));
    private Pose2d  blueReefZoneE         = new Pose2d(5.023, 2.736, new Rotation2d(30.0));
    private Pose2d  blueReefZoneF         = new Pose2d(5.239, 2.868, new Rotation2d(30.0));
    private Pose2d  blueReefZoneG         = new Pose2d(5.874, 3.839, new Rotation2d(90.0));
    private Pose2d  blueReefZoneH         = new Pose2d(5.874, 4.151, new Rotation2d(90.0));
    private Pose2d  blueReefZoneI         = new Pose2d(5.311, 5.158, new Rotation2d(150.0));
    private Pose2d  blueReefZoneJ         = new Pose2d(5.035, 5.290, new Rotation2d(150.0));
    private Pose2d  blueReefZoneK         = new Pose2d(3.956, 5.290, new Rotation2d(-150.0));
    private Pose2d  blueReefZoneL         = new Pose2d(3.692, 5.134, new Rotation2d(-150.0));

    private Pose2d  blueReefZoneAB        = new Pose2d(3.093, 4.091, new Rotation2d(90.0));
    private Pose2d  blueReefZoneCD        = new Pose2d(3.896, 2.736, new Rotation2d(150.0));
    private Pose2d  blueReefZoneEF        = new Pose2d(5.263, 2.844, new Rotation2d(-150.0));
    private Pose2d  blueReefZoneGH        = new Pose2d(5.874, 4.091, new Rotation2d(-90.0));
    private Pose2d  blueReefZoneIJ        = new Pose2d(5.154, 5.242, new Rotation2d(-30.0));
    private Pose2d  blueReefZoneKL        = new Pose2d(3.896, 0.8972, new Rotation2d(0.0));  //FIXME

    private Pose2d  blueReefFaceAB        = new Pose2d(8.34924, 7.0528, new Rotation2d(0.0));  //FIXME
    private Pose2d  blueReefFaceCD        = new Pose2d(8.34924, 5.9264, new Rotation2d(0.0));  //FIXME
    private Pose2d  blueReefFaceEF        = new Pose2d(8.34924, 4.25, new Rotation2d(0.0));    //FIXME
    private Pose2d  blueReefFaceGH        = new Pose2d(8.34924, 2.5736, new Rotation2d(0.0));  //FIXME
    private Pose2d  blueReefFaceIJ        = new Pose2d(8.34924, 0.8972, new Rotation2d(0.0));  //FIXME
    private Pose2d  blueReefFaceKL        = new Pose2d(8.34924, 0.8972, new Rotation2d(0.0));  //FIXME

    private double  redJoystickInversion     = -1.0;
    private Pose2d  redStartLeft             = new Pose2d(1.60, 6.95, new Rotation2d(Math.toRadians(48.0)));
    private Pose2d  redStartMiddle           = new Pose2d(1.60, 6.95, new Rotation2d(Math.toRadians(48.0)));
    private Pose2d  redStartRight            = new Pose2d(1.60, 6.95, new Rotation2d(Math.toRadians(48.0)));
    private Pose2d  redProcessor             = new Pose2d(3.25, 7.05, new Rotation2d(0.0));
    private Pose2d  redProcessorFace         = new Pose2d(3.25, 7.05, new Rotation2d(0.0));
    private Pose2d  redCoralStationLeft      = new Pose2d(2.75, 5.5, new Rotation2d(0.0));
    private Pose2d  redCoralStationLeftFace  = new Pose2d(2.75, 5.5, new Rotation2d(0.0));
    private Pose2d  redCoralStationRight     = new Pose2d(2.5, 4.25, new Rotation2d(0.0));
    private Pose2d  redCoralStationRightFace = new Pose2d(2.5, 4.25, new Rotation2d(0.0));

    private Pose2d  redReefZoneA          = new Pose2d(8.34924, 7.0528, new Rotation2d(0.0));
    private Pose2d  redReefZoneB          = new Pose2d(8.34924, 5.9264, new Rotation2d(0.0));
    private Pose2d  redReefZoneC          = new Pose2d(8.34924, 4.25, new Rotation2d(0.0));
    private Pose2d  redReefZoneD          = new Pose2d(8.34924, 2.5736, new Rotation2d(0.0));
    private Pose2d  redReefZoneE          = new Pose2d(8.34924, 0.8972, new Rotation2d(0.0));
    private Pose2d  redReefZoneF          = new Pose2d(8.34924, 0.8972, new Rotation2d(0.0));
    private Pose2d  redReefZoneG          = new Pose2d(8.34924, 7.0528, new Rotation2d(0.0));
    private Pose2d  redReefZoneH          = new Pose2d(8.34924, 5.9264, new Rotation2d(0.0));
    private Pose2d  redReefZoneI          = new Pose2d(8.34924, 4.25, new Rotation2d(0.0));
    private Pose2d  redReefZoneJ          = new Pose2d(8.34924, 2.5736, new Rotation2d(0.0));
    private Pose2d  redReefZoneK          = new Pose2d(8.34924, 0.8972, new Rotation2d(0.0));
    private Pose2d  redReefZoneL          = new Pose2d(8.34924, 0.8972, new Rotation2d(0.0));

    private Pose2d  redReefZoneAB         = new Pose2d(8.34924, 7.0528, new Rotation2d(0.0));
    private Pose2d  redReefZoneCD         = new Pose2d(8.34924, 5.9264, new Rotation2d(0.0));
    private Pose2d  redReefZoneEF         = new Pose2d(8.34924, 4.25, new Rotation2d(0.0));
    private Pose2d  redReefZoneGH         = new Pose2d(8.34924, 2.5736, new Rotation2d(0.0));
    private Pose2d  redReefZoneIJ         = new Pose2d(8.34924, 0.8972, new Rotation2d(0.0));
    private Pose2d  redReefZoneKL         = new Pose2d(8.34924, 0.8972, new Rotation2d(0.0));

    private Pose2d  redReefFaceAB         = new Pose2d(8.34924, 7.0528, new Rotation2d(0.0));
    private Pose2d  redReefFaceCD         = new Pose2d(8.34924, 5.9264, new Rotation2d(0.0));
    private Pose2d  redReefFaceEF         = new Pose2d(8.34924, 4.25, new Rotation2d(0.0));
    private Pose2d  redReefFaceGH         = new Pose2d(8.34924, 2.5736, new Rotation2d(0.0));
    private Pose2d  redReefFaceIJ         = new Pose2d(8.34924, 0.8972, new Rotation2d(0.0));
    private Pose2d  redReefFaceKL         = new Pose2d(8.34924, 0.8972, new Rotation2d(0.0));

    public Alliance current_alliance;
    public double   joystickInversion;
    public Pose2d   startLeft             = blueStartLeft;
    public Pose2d   startMiddle           = blueStartMiddle;
    public Pose2d   startRight            = blueStartRight;
    public Pose2d   processor             = blueProcessor;
    public Pose2d   processorFace         = blueProcessorFace;
    public Pose2d   coralStationLeft      = blueCoralStationLeft;
    public Pose2d   coralStationLeftFace  = blueCoralStationLeftFace;
    public Pose2d   coralStationRight     = blueCoralStationRight;
    public Pose2d   coralStationRightFace = blueCoralStationRightFace;
    public Pose2d   reefZoneA             = blueReefZoneA;
    public Pose2d   reefZoneB             = blueReefZoneB;
    public Pose2d   reefZoneC             = blueReefZoneC;
    public Pose2d   reefZoneD             = blueReefZoneD;
    public Pose2d   reefZoneE             = blueReefZoneE;
    public Pose2d   reefZoneF             = blueReefZoneF;
    public Pose2d   reefZoneG             = blueReefZoneG;
    public Pose2d   reefZoneH             = blueReefZoneH;
    public Pose2d   reefZoneI             = blueReefZoneI;
    public Pose2d   reefZoneJ             = blueReefZoneJ;
    public Pose2d   reefZoneK             = blueReefZoneK;
    public Pose2d   reefZoneL             = blueReefZoneL;
    public Pose2d   reefZoneAB            = blueReefZoneAB;
    public Pose2d   reefZoneCD            = blueReefZoneCD;
    public Pose2d   reefZoneEF            = blueReefZoneEF;
    public Pose2d   reefZoneGH            = blueReefZoneGH;
    public Pose2d   reefZoneIJ            = blueReefZoneIJ;
    public Pose2d   reefZoneKL            = blueReefZoneKL;
    public Pose2d   reefFaceAB            = blueReefFaceAB;
    public Pose2d   reefFaceCD            = blueReefFaceCD;
    public Pose2d   reefFaceEF            = blueReefFaceEF;
    public Pose2d   reefFaceGH            = blueReefFaceGH;
    public Pose2d   reefFaceIJ            = blueReefFaceIJ;
    public Pose2d   reefFaceKL            = blueReefFaceKL;

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
            joystickInversion = redJoystickInversion;
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
        System.out.println(alliance);
    }
}