package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.SubsystemConfigBase;
import frc.robot.helpers.Logger;

@Logged
public abstract class ObotSubsystemBase<TConfig extends SubsystemConfigBase> extends SubsystemBase {
    protected TConfig config;

    protected String  className;

    protected boolean verbosity;

    protected boolean isSimulation = !RobotBase.isReal();

    protected Logger  log;

    protected Boolean enabled;

    protected ObotSubsystemBase(TConfig config) {
        this.config    = config;
        this.enabled   = config.enabled;
        this.verbosity = config.verbose;
        this.className = this.getClass().getSimpleName();
        this.log       = Logger.getInstance(this.getClass());
    }

    public Boolean isEnabled() {
        return enabled;
    }

    public Boolean isDisabled() {
        return !enabled;
    }
}
