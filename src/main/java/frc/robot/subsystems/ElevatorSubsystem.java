package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 *
 */
@Logged
public class ElevatorSubsystem extends SubsystemBase {
    private class StowCommand extends Command {
        ElevatorSubsystem subsystem;

        public StowCommand(ElevatorSubsystem new_subsystem) {
            super();
            subsystem = new_subsystem;
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
            super.initialize();
            subsystem.setTarget(0);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            super.execute();
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
            super.end(interrupted);
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            super.isFinished();
            return at_target;
        }
    }

    private class GoToCommand extends Command {

        int               position;

        ElevatorSubsystem subsystem;

        public GoToCommand(ElevatorSubsystem new_subsystem, int new_position) {
            super();
            position  = new_position;
            subsystem = new_subsystem;
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
            super.initialize();
            subsystem.setTarget(position);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            super.execute();
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
            super.end(interrupted);
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            super.isFinished();
            return at_target;
        }
    }

    private SparkMax rightElevatorMotor;

    private SparkMax leftElevatorMotor;

    private int      target;

    private boolean  at_target;

    /**
    *
    */
    public ElevatorSubsystem() {
        rightElevatorMotor = new SparkMax(0, MotorType.kBrushless);
        rightElevatorMotor.setInverted(false);

        leftElevatorMotor = new SparkMax(6, MotorType.kBrushless);
        leftElevatorMotor.setInverted(false);

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public Command stow() {
        return new StowCommand(this);
    }

    public Command goTo(int position) {
        return new GoToCommand(this, position);
    }

    private void setTarget(int new_target) {
        target    = new_target;
        at_target = false;
    }

}
