package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.SubsystemsConfig;
import frc.robot.helpers.Logger;

@Logged
public abstract class ObotSubsystemBase extends SubsystemBase {
    protected SubsystemsConfig subsystemsConfig;

    protected String           className;

    protected boolean          verbosity;

    protected boolean          isSimulation = !RobotBase.isReal();

    protected Logger           log;

    protected Boolean          isEnabled    = true;

    protected ObotSubsystemBase(SubsystemsConfig subsystemsConfig) {
        this.subsystemsConfig = subsystemsConfig;
        this.verbosity        = subsystemsConfig.verboseOutput;
        this.className        = this.getClass().getSimpleName();
        this.log              = Logger.getInstance(this.getClass());
    }

    public Boolean isEnabled() {
        return isEnabled;
    }
}
