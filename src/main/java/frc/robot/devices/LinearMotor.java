package frc.robot.devices;

/**
 * The LinearMotor class represents a motor with positional control capabilities. It uses a SparkMax motor controller and provides methods to
 * configure and retrieve encoder information.
 * <p>
 * This class handles the conversion of target positions to a specified conversion factor and configures the motor controller with various settings
 * including soft limits, voltage compensation, and idle mode.
 * </p>
 * Note: This class assumes the use of a brushless motor.
 * </p>
 */
public class LinearMotor extends PositionalMotor {

    /**
     * Constructs a LinearMotor with the specified device ID and target position limits.
     *
     * @param deviceId              The ID of the motor device.
     * @param minimumTargetPosition The minimum target position in centimeters for the motor.
     * @param maximumTargetPosition The maximum target position in centimeters for the motor.
     */

    public LinearMotor(int deviceId, double minimumTargetPosition, double maximumTargetPosition) {
        super(deviceId, minimumTargetPosition, maximumTargetPosition, true);

        conversionFactor = ((8.0 * Math.PI) / 10.0) * 31.0/66.49;
    }

    /**
     * Retrieves the current position of the motor's encoder.
     *
     * @return the position of the encoder in degrees.
     */
    @Override
    public double getEncoderPosition() {
        return relativeEncoder.getPosition();
    }

    /**
     * Retrieves the current velocity of the motor's encoder.
     *
     * @return the velocity of the encoder in degrees per minute.
     */
    @Override
    public double getEncoderVelocity() {
        return relativeEncoder.getVelocity();
    }

}
