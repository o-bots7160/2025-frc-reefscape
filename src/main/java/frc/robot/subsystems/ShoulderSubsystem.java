package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.TestLoggerCommand;
import frc.robot.commands.manipulator.shoulder.RotateShoulderCommand;
import frc.robot.config.ShoulderSubsystemConfig;
import frc.robot.config.SubsystemsConfig;
import frc.robot.devices.PositionalMotor;

/**
 *
 */
@Logged
public class ShoulderSubsystem extends SetAndSeekSubsystemBase<ShoulderSubsystemConfig> {

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.25124, 5.4867, 0.67516);

    /**
     * Construct a new Shoulder Subsustem
     */
    public ShoulderSubsystem(SubsystemsConfig subsystemsConfig) {
        super(subsystemsConfig.shoulderSubsystem);
        if (checkDisabled()) {
            return;
        }

        // Configure motors
        var shoulderMotor = new PositionalMotor(config.motorCanId, minimumSetPoint, maximumSetPoint, config.rotationOffset);
        motors.put(0, new MotorData(shoulderMotor, "shoulderMotor"));
    }

    /**
     * Returns true if the shoulder is at an angle where it can be stowed
     *
     * @return True if the shoulder is at an angle where it can be stowed
     */
    public boolean isStowed() {
        if (checkDisabled()) {
            return false;
        }

        double degrees = getPrimaryMotor().getEncoderPosition();
        return (degrees > -91.0) && (degrees < -89.0);
    }

    /**
     * Creates a new Shoulder command to move to the desired angle
     *
     * @param degrees that the shoulder needs turn to
     * @return Command to move the shoulder
     */
    public Command shoulderCommand(double degrees) {
        if (checkDisabled()) {
            return new TestLoggerCommand("shoulderCommand method not called");
        }

        return new RotateShoulderCommand(this, degrees);
    }

    public Command shoulderConstant(double volts) {
        if (checkDisabled()) {
            return new TestLoggerCommand("shoulderConstant method not called");
        }

        return Commands.startEnd(() -> this.setConstant(volts), () -> this.stop());
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
