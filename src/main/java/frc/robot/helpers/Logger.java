package frc.robot.helpers;

/**
 * Logger is a utility class for logging messages with different levels of
 * severity. It supports verbose, debug, info, warning, and error messages. The
 * output is color-coded for better readability in the console and automatically
 * includes the class name to help diagnose where the log originated.
 */
public class Logger {
    /**
     * Returns an instance of Logger for the specified class with the given
     * verbosity setting.
     *
     * @param <T> the type of the class for which the Logger instance is being
     *            created
     * @param c   the Class object for which the Logger instance is being created
     * @return a new Logger instance for the specified class with the given
     *         verbosity setting
     */
    public static <T> Logger getInstance(Class<T> c) {
        String className = c.getName();
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
}
