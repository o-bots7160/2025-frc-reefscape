package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.TestLoggerCommand;
import frc.robot.commands.elevator.MoveElevatorCommand;
import frc.robot.config.ElevatorSubsystemConfig;
import frc.robot.config.SubsystemsConfig;
import frc.robot.devices.LinearMotor;

/**
 *
 */
@Logged
public class ElevatorSubsystem extends SetAndSeekSubsystemBase<ElevatorSubsystemConfig> {

    private ElevatorFeedforward feedforward = new ElevatorFeedforward(0.063246, 0.091149, 1.5186, 0.18462);

    /**
    *
    */
    public ElevatorSubsystem(SubsystemsConfig subsystemsConfig) {
        super(subsystemsConfig.elevatorSubsystem);
        if (checkDisabled()) {
            return;
        }

        // Configure motors
        var rightElevatorMotor = new LinearMotor(config.rightMotorCanId, minimumSetPoint, maximumSetPoint);
        var leftElevatorMotor  = new LinearMotor(config.leftMotorCanId, minimumSetPoint, maximumSetPoint);
        motors.put(0, new MotorData(rightElevatorMotor, "rightElevatorMotor"));
        motors.put(1, new MotorData(leftElevatorMotor, "leftElevatorMotor"));
    }

    public Command goToCommand(double position) {
        if (checkDisabled()) {
            return new TestLoggerCommand("goToCommand method not called");
        }

        return new MoveElevatorCommand(this, position);
    }

    public Command goToCommand(Supplier<Double> position) {
        if (checkDisabled()) {
            return new TestLoggerCommand("goToCommand method not called");
        }

        return new MoveElevatorCommand(this, position);
    }

    public Command moveConstantCommand(double volts) {
        return Commands.sequence(Commands.run(() -> this.setConstant(volts), this));
    }

    @Override
    protected double calculateVoltageWithVelocities(double currentVelocity, double nextVelocity) {
        return feedforward.calculateWithVelocities(currentVelocity, nextVelocity);
    }

    @Override
    protected double calcuateVoltage(double velocity) {
        return feedforward.calculate(velocity);
    }
}
