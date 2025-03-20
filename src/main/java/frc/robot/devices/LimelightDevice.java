package frc.robot.devices;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.helpers.LimelightHelpers;
import frc.robot.helpers.LimelightHelpers.PoseEstimate;
import frc.robot.helpers.Pose2dFilter;

public class LimelightDevice {
    Pose2dFilter      filter = new Pose2dFilter();
    String            name;
    boolean           isEnabled = false;

    NetworkTable      table;

    NetworkTableEntry tx;

    NetworkTableEntry ty;

    NetworkTableEntry ta;

    NetworkTableEntry tid;

    NetworkTableEntry tl;

    NetworkTableEntry cl;

    NetworkTableEntry botpose;

    public LimelightDevice(String limelightName ) {
        name    = limelightName;
        table   = NetworkTableInstance.getDefault().getTable(name);

        tx      = table.getEntry("tx");

        ty      = table.getEntry("ty");

        ta      = table.getEntry("ta");

        tid     = table.getEntry("tid");

        tl      = table.getEntry("tl");

        cl      = table.getEntry("cl");

        botpose = table.getEntry("botpose");

    }

    public String getName() {
        return name;
    }

    public Double getX() {
        return tx.getDouble(0.0);
    }

    public Double getY() {
        return ty.getDouble(0.0);
    }

    public Double getAprilTagId() {
        return tid.getDouble(0.0);
    }

    public Double getArea() {
        return ta.getDouble(0.0);
    }
    public void enable( boolean state ) {
        isEnabled = state;
    }

    /**
     * Gets the pose estimate of the robot based on a reading from the current Limelight
     * 
     * @param headingDegress The degree of the robot's current heading
     * @return An estimate based on any tag readings from the Limelight
     */
    public PoseEstimate getPoseEstimate(double headingDegress) {
        PoseEstimate mt2;
        putSmartDashboardData();

        if ( isEnabled ) {
            LimelightHelpers.SetRobotOrientation(name, headingDegress, 0.0, 0.0, 0.0, 0.0, 0);
            mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        } else {
            mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
            /*
            if ( filter.addData(mt2.pose, mt2.timestampSeconds ) ) {
                mt2.pose = filter.avgPose;
                mt2.timestampSeconds = filter.avgTime;
                mt2.tagCount = 1;
            }
                */
        }
        return mt2;
    }

    private void putSmartDashboardData() {
        SmartDashboard.putNumber(name + "/X", getX());
        SmartDashboard.putNumber(name + "/Y", getY());
        SmartDashboard.putNumber(name + "/AprilTagId", getAprilTagId());
        SmartDashboard.putNumber(name + "/Area", getArea());
    }
}
