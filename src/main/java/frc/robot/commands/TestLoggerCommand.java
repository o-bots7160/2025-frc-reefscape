package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.helpers.Logger;

public class TestLoggerCommand extends Command {
    private Logger log;

    public TestLoggerCommand(String name) {
        log = Logger.getInstance(name);
    }

    @Override
    public void execute() {
        log.debug("execute");
    }

    @Override
    public void initialize() {
        log.debug("initialize");
    }

    @Override
    public void end(boolean interrupted) {
        log.debug("end");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
