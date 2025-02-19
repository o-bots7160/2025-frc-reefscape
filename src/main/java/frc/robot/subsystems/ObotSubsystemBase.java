package frc.robot.subsystems;

import javax.naming.ConfigurationException;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ConfigurationLoader;
import frc.robot.config.SubsystemsConfig;
import frc.robot.helpers.Logger;

@Logged
public abstract class ObotSubsystemBase extends SubsystemBase {
    protected static SubsystemsConfig subsystemsConfig;

    protected String                  className;

    protected boolean                 verbosity;

    protected boolean                 isSimulation = !RobotBase.isReal();

    protected Logger                  log;

    protected ObotSubsystemBase() {
        this.className = this.getClass().getSimpleName();
        this.log       = Logger.getInstance(this.getClass());
        loadConfig();
    }

    protected void loadConfig() {
        try {
            // There might be a better way to do this, but we really only want to load the
            // config one time
            // if (subsystemsConfig == null) {
            subsystemsConfig = ConfigurationLoader.load("subsystems.json", SubsystemsConfig.class);
            verbosity        = subsystemsConfig.verboseOutput;
            // }
        } catch (ConfigurationException e) {
            log.error("Failed to load configuration: " + e.getMessage());
            e.printStackTrace();
        }
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
