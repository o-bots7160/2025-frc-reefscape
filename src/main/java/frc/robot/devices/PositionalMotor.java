package frc.robot.devices;

/**
 * The PositionalMotor class represents a motor with positional control capabilities. It uses a SparkMax motor controller and provides methods to
 * configure and retrieve encoder information.
 * <p>
 * This class handles the conversion of target positions to a specified conversion factor and configures the motor controller with various settings
 * including soft limits, voltage compensation, and idle mode.
 * </p>
 * Note: This class assumes the use of a brushless motor.
 * </p>
 */
public class PositionalMotor extends MotorBase {

    /**
     * Constructs a PositionalMotor with the specified device ID and target position limits.
     *
     * @param deviceId              The ID of the motor device.
     * @param minimumTargetPosition The minimum target position in degrees for the motor.
     * @param maximumTargetPosition The maximum target position in degrees for the motor.
     */
    public PositionalMotor(int deviceId, double minimumTargetPosition, double maximumTargetPosition) {
        super(deviceId, minimumTargetPosition, maximumTargetPosition, false, true);

    }

    /**
     * Constructs a PositionalMotor with the specified device ID and target position limits.
     *
     * @param deviceId              The ID of the motor device.
     * @param minimumTargetPosition The minimum target position in degrees for the motor.
     * @param maximumTargetPosition The maximum target position in degrees for the motor.
     */
    public PositionalMotor(int deviceId, double minimumTargetPosition, double maximumTargetPosition, boolean isInverted) {
        super(deviceId, minimumTargetPosition, maximumTargetPosition, isInverted, true);

    }

}
