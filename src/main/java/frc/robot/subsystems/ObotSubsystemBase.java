package frc.robot.subsystems;

import javax.naming.ConfigurationException;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ConfigurationLoader;
import frc.robot.config.SubsystemsConfig;

@Logged
public abstract class ObotSubsystemBase extends SubsystemBase {
    protected static SubsystemsConfig subsystemsConfig;

    protected String                  className;

    protected boolean                 verbosity;

    protected boolean                 isSimulation = !RobotBase.isReal();

    protected ObotSubsystemBase() {
        this.className = this.getClass().getSimpleName();
        loadConfig();
    }

    protected void loadConfig() {
        try {
            // There might be a better way to do this, but we really only want to load the
            // config one time
            if (subsystemsConfig == null) {
                subsystemsConfig = ConfigurationLoader.load("subsystems.json", SubsystemsConfig.class);
                verbosity        = subsystemsConfig.verboseOutput;
            }
        } catch (ConfigurationException e) {
            logError("Failed to load configuration: " + e.getMessage());
            e.printStackTrace();
        }
    }

    protected void logVerbose(String message) {
        if (verbosity) {
            System.out.println("\u001B[90mVERBOSE: " + className + ": " + message + "\u001B[0m");
        }
    }

    protected void logDebug(String message) {
        if (verbosity) {
            System.out.println("\u001B[37mDEBUG: " + className + ": " + message + "\u001B[0m");
        }
    }

    protected void logInfo(String message) {
        System.out.println("\u001B[37mINFO: " + className + ": " + message + "\u001B[0m");
    }

    protected void logWarning(String message) {
        System.out.println("\u001B[33mWARN: " + className + ": " + message + "\u001B[0m");
    }

    protected void logError(String message) {
        System.err.println("\u001B[31mERROR: " + className + ": " + message + "\u001B[0m");
    }

    protected void putDashboardNumber(String name, double value) {
        SmartDashboard.putNumber(className + '/' + name, value);
    }

    protected void putDashboardNumberVerbose(String name, double value) {
        if (verbosity) {
            putDashboardNumber(name, value);
        }
    }

    protected void putDashboardString(String name, String value) {
        SmartDashboard.putString(className + '/' + name, value);
    }

    protected void putDashboardStringVerbose(String name, String value) {
        if (verbosity) {
            putDashboardString(name, value);
        }
    }

    protected void putDashboardBoolean(String name, Boolean value) {
        SmartDashboard.putBoolean(className + '/' + name, value);
    }

    protected void putDashboardBooleanVerbose(String name, Boolean value) {
        if (verbosity) {
            putDashboardBoolean(name, value);
        }
    }
}
