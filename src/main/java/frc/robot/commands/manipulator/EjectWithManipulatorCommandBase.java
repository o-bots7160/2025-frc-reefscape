
package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.IntakeSubsystemConfigBase;
import frc.robot.subsystems.IntakeSubsystemBase;

public abstract class EjectWithManipulatorCommandBase<T extends IntakeSubsystemBase<TConfig>, TConfig extends IntakeSubsystemConfigBase> extends Command {
    private T subsystem;

    public EjectWithManipulatorCommandBase(T subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.eject();
    }

    @Override
    public boolean isFinished() {
        return !subsystem.hasItem();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }
}
