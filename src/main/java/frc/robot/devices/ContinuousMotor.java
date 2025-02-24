package frc.robot.devices;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.helpers.Logger;

public class ContinuousMotor {

    private double   rotationsToDegreesConversionFactor = 360.0;

    private SparkMax motor;

    private Logger   log                                = Logger.getInstance(this.getClass());

    public ContinuousMotor(int deviceId) {

        motor = new SparkMax(deviceId, MotorType.kBrushless);
        log.verbose("Configuring brushless SparkMax motor with device ID " + deviceId);

        SparkMaxConfig config = new SparkMaxConfig();

        // Basic config
        config.inverted(false).voltageCompensation(12.0).idleMode(IdleMode.kBrake);

        // Absolute encoder config
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        config.absoluteEncoder.inverted(false)
                // Setting conversion factors
                .positionConversionFactor(rotationsToDegreesConversionFactor)
                .velocityConversionFactor(rotationsToDegreesConversionFactor)
                // center output range: -0.5 to 0.5 rather than 0.0 to 1.0
                .zeroCentered(true)
                // TODO: Should this be calibrated straight down?
                .zeroOffset(0.0);

        // load the configuration into the motor
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /**
     * Retrieves the current position of the motor's encoder.
     *
     * @return the position of the encoder in degrees.
     */
    public double getEncoderPosition() {
        var encoder = motor.getAbsoluteEncoder();
        return encoder.getPosition();
    }

    /**
     * Retrieves the current velocity of the motor's encoder.
     *
     * @return the velocity of the encoder in degrees per minute.
     */
    public double getEncoderVelocity() {
        var encoder = motor.getAbsoluteEncoder();
        return encoder.getVelocity();
    }

    /**
     * Sets the voltage of the motor.
     *
     * @param voltage the Voltage object representing the desired voltage to be set.
     */
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.baseUnitMagnitude());
    }

    /**
     * Sets the voltage of the motor.
     *
     * @param double the desired voltage to be set.
     */
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
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
