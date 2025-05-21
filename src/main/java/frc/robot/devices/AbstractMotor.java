package frc.robot.devices;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.helpers.Logger;

public abstract class AbstractMotor implements Motor {

    protected double         conversionFactor;

    protected double         maximumTargetPosition;

    protected double         minimumTargetPosition;

    protected double         voltageCompensation = 12.0;

    protected Logger         log                 = Logger.getInstance(this.getClass());

    protected SparkMax       motor;

    protected SparkMaxConfig config;

    protected String         name;

    private double           lastSetVoltage;

    public AbstractMotor(String name, int deviceId) {
        // TODO: for some motors we don't have target positions, probably should allow null
        this(name, deviceId, 0, Double.MAX_VALUE, 1);
    }

    public AbstractMotor(String name, int deviceId, double minimumTargetPosition, double maximumTargetPosition) {
        this(name, deviceId, minimumTargetPosition, maximumTargetPosition, 360.0);
    }

    public AbstractMotor(String name, int deviceId, double minimumTargetPosition, double maximumTargetPosition, double conversionFactor) {
        this.name                  = name;
        this.minimumTargetPosition = minimumTargetPosition;
        this.maximumTargetPosition = maximumTargetPosition;
        this.conversionFactor      = conversionFactor;

        motor                      = new SparkMax(deviceId, MotorType.kBrushless);
        log.verbose("Configuring brushless SparkMax motor with device ID " + deviceId);

        // Configure the motor with default settings
        config = new SparkMaxConfig();
        config.voltageCompensation(voltageCompensation);

        // Call to custom configure method
        config = configureMotor(config);

        // load the configuration into the motor
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the voltage of the motor.
     *
     * @param voltage the Voltage object representing the desired voltage to be set.
     */
    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.baseUnitMagnitude());
    }

    /**
     * Sets the voltage of the motor.
     *
     * @param double the desired voltage to be set.
     */
    @Override
    public void setVoltage(double voltage) {
        this.lastSetVoltage = voltage;
        motor.setVoltage(voltage);
    }

    /**
     * Sets the speed of the motor.
     *
     * @param speed the double object representing the desired speed to be set.
     */
    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    /**
     * Retrieves the converted maximum target position for the motor.
     *
     * @return The converted maximum target position as a double.
     */
    @Override
    public double getMaximumTargetPosition() {
        return maximumTargetPosition;
    }

    /**
     * Retrieves the converted minimum target position for the motor.
     *
     * @return The converted minimum target position as a double.
     */
    @Override
    public double getMinimumTargetPosition() {
        return minimumTargetPosition;
    }

    /**
     * Retrieves the current position of the motor's encoder.
     *
     * @return the position of the encoder in degrees.
     */
    @Override
    public abstract double getEncoderPosition();

    /**
     * Retrieves the current velocity of the motor's encoder.
     *
     * @return the velocity of the encoder in degrees per minute.
     */
    @Override
    public abstract double getEncoderVelocity();

    /**
     * Retrieves the voltage applied to the motor.
     *
     * @return the voltage applied to the motor as a Voltage object.
     */
    @Override
    public Voltage getVoltage() {
        return Units.Volts.of(motor.getBusVoltage() * motor.getAppliedOutput());
    }

    /**
     * Stops the motor
     */
    @Override
    public void stop() {
        motor.stopMotor();
    }

    /**
     * Gets the underlying motor object (mostly for testing)
     */
    @Override
    public SparkMax getMotor() {
        return motor;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(this.name);
        builder.addDoubleProperty("lastSetVoltage", () -> lastSetVoltage, null);
        // TODO: need to validate how this impacts performance; may want to cache this somehow
        builder.addDoubleProperty("currentVoltage", () -> getVoltage().baseUnitMagnitude(), null);
    }

    /**
     * Configures the motor using the provided {@link SparkMaxConfig} object. Implementations should apply any necessary settings or modifications to
     * the configuration before returning it. This method is intended to be overridden by subclasses to provide device-specific configuration logic.
     *
     * @param config the initial {@code SparkMaxConfig} to be configured
     * @return the configured {@code SparkMaxConfig} instance
     */
    protected abstract SparkMaxConfig configureMotor(SparkMaxConfig config);
}
