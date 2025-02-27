package frc.robot.devices;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;

/**
 * The PositionalMotor class represents a motor with positional control
 * capabilities. It uses a SparkMax motor controller and provides methods to
 * configure and retrieve encoder information.
 * <p>
 * This class handles the conversion of target positions to a specified
 * conversion factor and configures the motor controller with various settings
 * including soft limits, voltage compensation, and idle mode.
 * </p>
 * Note: This class assumes the use of a brushless motor.
 * </p>
 */
public class PositionalMotor extends ContinuousMotor {

    /**
     * Constructs a PositionalMotor with the specified device ID and target position
     * limits.
     *
     * @param deviceId              The ID of the motor device.
     * @param minimumTargetPosition The minimum target position in degrees for the
     *                              motor.
     * @param maximumTargetPosition The maximum target position in degrees for the
     *                              motor.
     */
    public PositionalMotor(int deviceId, double minimumTargetPosition, double maximumTargetPosition) {
        super(deviceId, minimumTargetPosition, maximumTargetPosition);

    }

    /**
     * Retrieves the current position of the motor's encoder.
     *
     * @return the position of the encoder in degrees.
     */
    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    /**
     * Retrieves the current velocity of the motor's encoder.
     *
     * @return the velocity of the encoder in degrees per minute.
     */
    public double getEncoderVelocity() {
        return encoder.getVelocity();
    }

    /**
     * Retrieves the voltage applied to the motor.
     *
     * @return the voltage applied to the motor as a Voltage object.
     */
    public Voltage getVoltage() {
        return Units.Volts.of(motor.getBusVoltage() * motor.getAppliedOutput());
    }

}
