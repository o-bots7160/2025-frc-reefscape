package frc.robot.helpers;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Logger is a utility class for logging messages with different levels of
 * severity. It supports verbose, debug, info, warning, and error messages. The
 * output is color-coded for better readability in the console and automatically
 * includes the class name to help diagnose where the log originated.
 */
public class Logger {
    /**
     * Returns an instance of Logger for the specified class
     *
     * @param <T> the type of the class for which the Logger instance is being
     *            created
     * @param c   the Class object for which the Logger instance is being created
     * @return a new Logger instance for the specified class with the given
     *         verbosity setting
     */
    public static <T> Logger getInstance(Class<T> c) {
        return getInstance(c.getSimpleName());
    }

    /**
     * Returns an instance of Logger for the specified name
     *
     * @param <T>       the type of the class for which the Logger instance is being
     *                  created
     * @param className the name to scope the logger instance to
     * @return a new Logger instance for the specified class with the given
     *         verbosity setting
     */
    public static Logger getInstance(String className) {
        return new Logger(className);
    }

    private String  className;

    private Boolean verboseOutput;

    protected Logger(String className) {
        this.className     = className;
        // TODO: pull the config on this
        this.verboseOutput = true;
    }

    /**
     * Logs a verbose message to the console if verbose output is enabled. The
     * message is prefixed with "VERBOSE:" and the class name, and is displayed in
     * gray color.
     *
     * @param message The message to be logged.
     */
    public void verbose(String message) {
        if (verboseOutput) {
            System.out.println("\u001B[90mVERBOSE: " + className + ": " + message + "\u001B[0m");
        }
    }

    /**
     * Logs a debug message to the console if verbose output is enabled. The message
     * is prefixed with "DEBUG:" and the class name, and is displayed in white
     * color.
     *
     * @param message The debug message to be logged.
     */
    public void debug(String message) {
        if (verboseOutput) {
            System.out.println("\u001B[37mDEBUG: " + className + ": " + message + "\u001B[0m");
        }
    }

    /**
     * Logs an informational message to the console. The message is prefixed with
     * "INFO:" and the class name, and is displayed in white color.
     *
     * @param message The informational message to be logged.
     */
    public void info(String message) {
        System.out.println("\u001B[37mINFO: " + className + ": " + message + "\u001B[0m");
    }

    /**
     * Logs a warning message to the console. The message is prefixed with "WARN:"
     * and the class name, and is displayed in yellow color.
     *
     * @param message The warning message to be logged.
     */
    public void warning(String message) {
        System.out.println("\u001B[33mWARN: " + className + ": " + message + "\u001B[0m");
    }

    /**
     * Logs a warning message to the standard error console. The message is prefixed
     * with "ERROR:" and the class name, and is displayed in red color.
     *
     * @param message The error message to be logged.
     */
    public void error(String message) {
        System.err.println("\u001B[31mERROR: " + className + ": " + message + "\u001B[0m");
    }

    /**
     * Logs a message to the SmartDashboard with a specified key and value.
     *
     * @param key   The key under which the value will be stored. The key will be
     *              prefixed with the class name followed by a '/'.
     * @param value The double value to be logged to the SmartDashboard.
     */
    public void dashboard(String key, boolean value) {
        SmartDashboard.putBoolean(className + '/' + key, value);
        debug(key + ": " + value);
    }

    /**
     * Logs a message to the SmartDashboard with a specified key and value.
     *
     * @param key   The key under which the value will be stored. The key will be
     *              prefixed with the class name followed by a '/'.
     * @param value The double value to be logged to the SmartDashboard.
     */
    public void dashboard(String key, double value) {
        SmartDashboard.putNumber(className + '/' + key, value);
        debug(key + ": " + value);
    }

    /**
     * Logs a message to the SmartDashboard with a specified key and value.
     *
     * @param key   The key under which the value will be stored. The key will be
     *              prefixed with the class name followed by a '/'.
     * @param value The Sendable value to be logged to the SmartDashboard.
     */
    public void dashboard(String key, Sendable value) {
        SmartDashboard.putData(className + '/' + key, value);
        debug(key + ": sendable logged to dashboard");
    }

    /**
     * Logs a message to the SmartDashboard with a specified key and value.
     *
     * @param key   The key under which the value will be stored. The key will be
     *              prefixed with the class name followed by a '/'.
     * @param value The string value to be logged to the SmartDashboard.
     */
    public void dashboard(String key, String value) {
        SmartDashboard.putString(className + '/' + key, value);
        debug(key + ": " + value);
    }

    /**
     * Logs a message to the SmartDashboard with a specified key and value. This
     * method will only log the message if verbose output is enabled.
     *
     * @param key   The key under which the value will be stored. The key will be
     *              prefixed with the class name followed by a '/'.
     * @param value the value to be logged
     */
    public void dashboardVerbose(String key, boolean value) {
        if (verboseOutput) {
            dashboard(key, value);
        }
    }

    /**
     * Logs a message to the SmartDashboard with a specified key and value. This
     * method will only log the message if verbose output is enabled.
     *
     * @param key   The key under which the value will be stored. The key will be
     *              prefixed with the class name followed by a '/'.
     * @param value the value to be logged
     */
    public void dashboardVerbose(String key, double value) {
        if (verboseOutput) {
            dashboard(key, value);
        }
    }

    /**
     * Logs a message to the SmartDashboard with a specified key and value. This
     * method will only log the message if verbose output is enabled.
     *
     * @param key   The key under which the value will be stored. The key will be
     *              prefixed with the class name followed by a '/'.
     * @param value the object to be logged
     */
    public void dashboardVerbose(String key, Sendable value) {
        if (verboseOutput) {
            dashboard(key, value);
        }
    }

    /**
     * Logs a message to the SmartDashboard with a specified key and value. This
     * method will only log the message if verbose output is enabled.
     *
     * @param key   The key under which the value will be stored. The key will be
     *              prefixed with the class name followed by a '/'.
     * @param value the value to be logged
     */
    public void dashboardVerbose(String key, String value) {
        if (verboseOutput) {
            dashboard(key, value);
        }
    }
}
