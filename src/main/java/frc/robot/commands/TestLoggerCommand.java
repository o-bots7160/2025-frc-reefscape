package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class TestLoggerCommand extends Command {
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        System.out.println("execute");
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        System.out.println("initialize");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("end");
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
