package frc.robot.config;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Logged
public class DriveBaseSubsystemConfig extends SubsystemConfigBase {
    public double maximumSpeedInFeet;

    public double thetaControllerTolerance;

    public double thetaControllerPidKp;

    public double thetaControllerPidKi;

    public double thetaControllerPidKd;

    public double getThetaControllerTolerance() {
        var v = SmartDashboard.getNumber("DriveBaseSubsystemConfig-thetaControllerTolerance", thetaControllerTolerance);
        return v;
    }

    public double getThetaControllerPidKp() {
        var v = SmartDashboard.getNumber("DriveBaseSubsystemConfig-thetaControllerPidKp", thetaControllerPidKp);
        return v;
    }

    public double getThetaControllerPidKi() {
        var v = SmartDashboard.getNumber("DriveBaseSubsystemConfig-thetaControllerPidKi", thetaControllerPidKi);
        return v;
    }

    public double getThetaControllerPidKd() {
        var v = SmartDashboard.getNumber("DriveBaseSubsystemConfig-thetaControllerPidKd", thetaControllerPidKd);
        return v;
    }

    public double getMaximumSpeedInFeet() {
        var v = SmartDashboard.getNumber("DriveBaseSubsystemConfig-maximumSpeedInFeet", maximumSpeedInFeet);
        return v;
    }

    public double getMaximumSpeedInMeters() {
        return Units.feetToMeters(getMaximumSpeedInFeet());
    }
}

abstract class SubsystemConfigBase {
}
