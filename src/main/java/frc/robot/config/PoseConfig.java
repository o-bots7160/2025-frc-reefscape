package frc.robot.config;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

@Logged
public class PoseConfig {
    public double x;

    public double y;

    public double rotation;

    public Pose2d getPose() {
        var pose = new Pose2d(x, y, new Rotation2d(Math.toRadians(rotation)));
        return pose;
    }
}