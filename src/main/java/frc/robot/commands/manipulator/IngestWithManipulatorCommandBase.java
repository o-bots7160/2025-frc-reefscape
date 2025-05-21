package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.AbstractIntakeSubsystemConfig;
import frc.robot.subsystems.AbstractIntakeSubsystem;

public abstract class IngestWithManipulatorCommandBase<T extends AbstractIntakeSubsystem<TConfig>, TConfig extends AbstractIntakeSubsystemConfig> extends Command {
    private T subsystem;

    public IngestWithManipulatorCommandBase(T subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.ingest();
    }

    @Override
    public boolean isFinished() {
        return subsystem.hasItem();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }
}
