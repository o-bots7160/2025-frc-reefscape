package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceLandmarks {
    private double  blueJoystickInversion = 1.0;
    private Pose2d  blueStartLeft         = new Pose2d(1.60, 6.95, new Rotation2d(Math.toRadians(48.0)));
    private Pose2d  blueStartMiddle       = new Pose2d(1.60, 6.95, new Rotation2d(Math.toRadians(48.0)));
    private Pose2d  blueStartRight        = new Pose2d(1.60, 6.95, new Rotation2d(Math.toRadians(48.0)));
    private Pose2d  blueProcessor         = new Pose2d(3.25, 7.05, new Rotation2d(0.0));
    private Pose2d  blueCoralStationLeft  = new Pose2d(2.75, 5.5, new Rotation2d(0.0));
    private Pose2d  blueCoralStationRight = new Pose2d(2.5, 4.25, new Rotation2d(0.0));

    private Pose2d  blueReefZoneA         = new Pose2d(8.34924, 7.0528, new Rotation2d(0.0));
    private Pose2d  blueReefZoneB         = new Pose2d(8.34924, 5.9264, new Rotation2d(0.0));
    private Pose2d  blueReefZoneC         = new Pose2d(8.34924, 4.25, new Rotation2d(0.0));
    private Pose2d  blueReefZoneD         = new Pose2d(8.34924, 2.5736, new Rotation2d(0.0));
    private Pose2d  blueReefZoneE         = new Pose2d(8.34924, 0.8972, new Rotation2d(0.0));
    private Pose2d  blueReefZoneF         = new Pose2d(8.34924, 0.8972, new Rotation2d(0.0));
    private Pose2d  blueReefZoneG         = new Pose2d(8.34924, 7.0528, new Rotation2d(0.0));
    private Pose2d  blueReefZoneH         = new Pose2d(8.34924, 5.9264, new Rotation2d(0.0));
    private Pose2d  blueReefZoneI         = new Pose2d(8.34924, 4.25, new Rotation2d(0.0));
    private Pose2d  blueReefZoneJ         = new Pose2d(8.34924, 2.5736, new Rotation2d(0.0));
    private Pose2d  blueReefZoneK         = new Pose2d(8.34924, 0.8972, new Rotation2d(0.0));
    private Pose2d  blueReefZoneL         = new Pose2d(8.34924, 0.8972, new Rotation2d(0.0));

    private Pose2d  blueReefZoneAB        = new Pose2d(8.34924, 7.0528, new Rotation2d(0.0));
    private Pose2d  blueReefZoneCD        = new Pose2d(8.34924, 5.9264, new Rotation2d(0.0));
    private Pose2d  blueReefZoneEF        = new Pose2d(8.34924, 4.25, new Rotation2d(0.0));
    private Pose2d  blueReefZoneGH        = new Pose2d(8.34924, 2.5736, new Rotation2d(0.0));
    private Pose2d  blueReefZoneIJ        = new Pose2d(8.34924, 0.8972, new Rotation2d(0.0));
    private Pose2d  blueReefZoneKL        = new Pose2d(8.34924, 0.8972, new Rotation2d(0.0));

    private double  redJoystickInversion  = -1.0;
    private Pose2d  redStartLeft          = new Pose2d(1.60, 6.95, new Rotation2d(Math.toRadians(48.0)));
    private Pose2d  redStartMiddle        = new Pose2d(1.60, 6.95, new Rotation2d(Math.toRadians(48.0)));
    private Pose2d  redStartRight         = new Pose2d(1.60, 6.95, new Rotation2d(Math.toRadians(48.0)));
    private Pose2d  redProcessor          = new Pose2d(3.25, 7.05, new Rotation2d(0.0));
    private Pose2d  redCoralStationLeft   = new Pose2d(2.75, 5.5, new Rotation2d(0.0));
    private Pose2d  redCoralStationRight  = new Pose2d(2.5, 4.25, new Rotation2d(0.0));

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

    public Alliance current_alliance;
    public double   joystickInversion;
    public Pose2d   startLeft             = blueStartLeft;
    public Pose2d   startMiddle           = blueStartMiddle;
    public Pose2d   startRight            = blueStartRight;
    public Pose2d   processor             = blueProcessor;
    public Pose2d   coralStationLeft      = blueCoralStationLeft;
    public Pose2d   coralStationRight     = blueCoralStationRight;
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

    public void newAlliance(Alliance alliance) {
        current_alliance = alliance;
        if (alliance == Alliance.Blue) {
            joystickInversion = blueJoystickInversion;
            startLeft         = blueStartLeft;
            startMiddle       = blueStartMiddle;
            startRight        = blueStartRight;
            processor         = blueProcessor;
            coralStationLeft  = blueCoralStationLeft;
            coralStationRight = blueCoralStationRight;
            reefZoneA         = blueReefZoneA;
            reefZoneB         = blueReefZoneB;
            reefZoneC         = blueReefZoneC;
            reefZoneD         = blueReefZoneD;
            reefZoneE         = blueReefZoneE;
            reefZoneF         = blueReefZoneF;
            reefZoneG         = blueReefZoneG;
            reefZoneH         = blueReefZoneH;
            reefZoneI         = blueReefZoneI;
            reefZoneJ         = blueReefZoneJ;
            reefZoneK         = blueReefZoneK;
            reefZoneL         = blueReefZoneL;
            reefZoneAB        = blueReefZoneAB;
            reefZoneCD        = blueReefZoneCD;
            reefZoneEF        = blueReefZoneEF;
            reefZoneGH        = blueReefZoneGH;
            reefZoneIJ        = blueReefZoneIJ;
            reefZoneKL        = blueReefZoneKL;
        } else {
            joystickInversion = redJoystickInversion;
            startLeft         = redStartLeft;
            startMiddle       = redStartMiddle;
            startRight        = redStartRight;
            processor         = redProcessor;
            coralStationLeft  = redCoralStationLeft;
            coralStationRight = redCoralStationRight;
            reefZoneA         = redReefZoneA;
            reefZoneB         = redReefZoneB;
            reefZoneC         = redReefZoneC;
            reefZoneD         = redReefZoneD;
            reefZoneE         = redReefZoneE;
            reefZoneF         = redReefZoneF;
            reefZoneG         = redReefZoneG;
            reefZoneH         = redReefZoneH;
            reefZoneI         = redReefZoneI;
            reefZoneJ         = redReefZoneJ;
            reefZoneK         = redReefZoneK;
            reefZoneL         = redReefZoneL;
            reefZoneAB        = redReefZoneAB;
            reefZoneCD        = redReefZoneCD;
            reefZoneEF        = redReefZoneEF;
            reefZoneGH        = redReefZoneGH;
            reefZoneIJ        = redReefZoneIJ;
            reefZoneKL        = redReefZoneKL;
        }
        System.out.println(alliance);
    }
}