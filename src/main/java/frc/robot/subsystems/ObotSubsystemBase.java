package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.SubsystemConfigBase;
import frc.robot.helpers.Logger;

@Logged
public abstract class ObotSubsystemBase<TConfig extends SubsystemConfigBase> extends SubsystemBase {
    protected static double kDt          = 0.02;

    protected TConfig       config;

    protected String        className;

    protected boolean       verbose;

    protected boolean       isSimulation = !RobotBase.isReal();

    protected Logger        log;

    protected boolean       enabled;

    protected ObotSubsystemBase(TConfig config) {
        this.config    = config;
        this.enabled   = config.enabled;
        this.verbose   = config.verbose;
        this.className = this.getClass().getSimpleName();
        this.log       = Logger.getInstance(this.getClass(), verbose);
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isDisabled() {
        return !enabled;
    }

    public boolean checkDisabled() {
        if (isDisabled()) {
            // NOTE: this is expensive, but really should only happen in debug scenarios
            StackTraceElement[] stackTrace = Thread.currentThread().getStackTrace();
            String              methodName = stackTrace[2].getMethodName();
            log.verbose("Subsystem is disabled; call to " + methodName + " ignored.");

            return true;
        }

        return false;
    }
}
