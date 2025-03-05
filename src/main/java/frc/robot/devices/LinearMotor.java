package frc.robot.devices;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
public class LinearMotor extends MotorBase {

    // This was calculated off measuring from encoder to tape measure
    private static final double   conversionFactor = 40.0/81.503;

    /**
     * Constructs a LinearMotor with the specified device ID and target position limits.
     *
     * @param deviceId              The ID of the motor device.
     * @param minimumTargetPosition The minimum target position in centimeters for the motor.
     * @param maximumTargetPosition The maximum target position in centimeters for the motor.
     */

    public LinearMotor(int deviceId, double minimumTargetPosition, double maximumTargetPosition) {
        super(deviceId, minimumTargetPosition, maximumTargetPosition, conversionFactor, true, false, IdleMode.kBrake);

    }

}
