package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.config.ShoulderSubsystemConfig;
import frc.robot.config.SubsystemsConfig;
import frc.robot.devices.PositionalMotor;

/**
 *
 */
@Logged
public class ShoulderSubsystem extends SetAndSeekSubsystemBase<ShoulderSubsystemConfig> {

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.58241, 8.2131, 1.0563);

    /**
     * Construct a new Shoulder Subsustem
     */
    public ShoulderSubsystem(SubsystemsConfig subsystemsConfig) {
        super(subsystemsConfig.shoulderSubsystem);
        if (checkDisabled()) {
            return;
        }

        // Configure motors
        var shoulderMotor = new PositionalMotor(config.motorCanId, minimumSetPoint, maximumSetPoint, true, config.rotationOffset);
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

    @Override
    protected double calculateVoltageWithVelocities(double currentVelocity, double nextVelocity) {
        return feedforward.calculateWithVelocities(currentVelocity, nextVelocity);
    }

    @Override
    protected double calculateVoltage(double velocity) {
        return feedforward.calculate(velocity);
    }

}
