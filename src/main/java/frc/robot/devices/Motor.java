package frc.robot.devices;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;

public interface Motor extends Sendable {

    /**
     * Sets the voltage of the motor.
     *
     * @param voltage the Voltage object representing the desired voltage to be set.
     */
    void setVoltage(Voltage voltage);

    /**
     * Sets the voltage of the motor.
     *
     * @param double the desired voltage to be set.
     */
    void setVoltage(double voltage);

    /**
     * Sets the speed of the motor.
     *
     * @param speed the double object representing the desired speed to be set.
     */
    void setSpeed(double speed);

    /**
     * Retrieves the converted maximum target position for the motor.
     *
     * @return The converted maximum target position as a double.
     */
    double getMaximumTargetPosition();

    /**
     * Retrieves the converted minimum target position for the motor.
     *
     * @return The converted minimum target position as a double.
     */
    double getMinimumTargetPosition();

    /**
     * Retrieves the current position of the motor's encoder.
     *
     * @return the position of the encoder in degrees.
     */
    double getEncoderPosition();

    /**
     * Retrieves the current velocity of the motor's encoder.
     *
     * @return the velocity of the encoder in degrees per minute.
     */
    double getEncoderVelocity();

    /**
     * Retrieves the voltage applied to the motor.
     *
     * @return the voltage applied to the motor as a Voltage object.
     */
    Voltage getVoltage();

    /**
     * Stops the motor
     */
    void stop();

    /**
     * Gets the underlying motor object (mostly for testing)
     */
    SparkMax getMotor();

}