package frc.robot.devices;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
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

public class MotorBase {

    protected double          voltageCompensation = 12.0;

    protected AbsoluteEncoder absoluteEncoder;

    protected double          maximumTargetPosition;

    protected double          minimumTargetPosition;

    protected SparkMax        motor;

    protected Logger          log                 = Logger.getInstance(this.getClass());

    protected RelativeEncoder relativeEncoder;

    protected IdleMode        idleMode;

    private boolean           useAbsoluteEncoder;

    public MotorBase(int deviceId, double minimumTargetPosition, double maximumTargetPosition, double conversionFactor, boolean isInverted,
            boolean useAbsoluteEncoder, IdleMode idleMode, double offset) {
        this.minimumTargetPosition = minimumTargetPosition;
        this.maximumTargetPosition = maximumTargetPosition;
        this.useAbsoluteEncoder    = useAbsoluteEncoder;
        this.idleMode              = idleMode;

        motor                      = new SparkMax(deviceId, MotorType.kBrushless);
        log.verbose("Configuring brushless SparkMax motor with device ID " + deviceId);

        SparkMaxConfig config = new SparkMaxConfig();

        // Basic config
        config.inverted(isInverted).voltageCompensation(voltageCompensation).idleMode(idleMode);

        // Configure encoder
        if (useAbsoluteEncoder) {
            config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
            config.absoluteEncoder.inverted(isInverted)
                    // Setting conversion factors
                    .positionConversionFactor(conversionFactor).velocityConversionFactor(conversionFactor)
                    // center output range: -0.5 to 0.5 rather than 0.0 to 1.0
                    .zeroCentered(true);
            if (offset != 0) {
                double offsetInRotations = offset/conversionFactor;
                config.absoluteEncoder.zeroOffset(offsetInRotations);
            } else {
                config.absoluteEncoder.zeroOffset(0);
            }

        } else {
            config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            config.encoder
                    // Setting conversion factors
                    .positionConversionFactor(conversionFactor).velocityConversionFactor(conversionFactor / 60.0);
        }

        // Soft limit config
        config.softLimit
                // Setting the forward soft limit with the conversion factor applied
                .forwardSoftLimit(maximumTargetPosition).forwardSoftLimitEnabled(false)
                // setting the reverse soft limit with the conversion factor applied
                .reverseSoftLimit(minimumTargetPosition).reverseSoftLimitEnabled(false);

        // load the configuration into the motor
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // assign encoders
        relativeEncoder = motor.getEncoder();

        if (useAbsoluteEncoder) {
            absoluteEncoder = motor.getAbsoluteEncoder();
        }

    }

    /**
     * Sets the speed of the motor.
     *
     * @param double the desired speed to be set (between -1 and 1).
     */
    public void setSpeed(double speed) {
        motor.set(speed);
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
     * Retrieves the converted maximum target position for the motor.
     *
     * @return The converted maximum target position as a double.
     */
    public double getMaximumTargetPosition() {
        return maximumTargetPosition;
    }

    /**
     * Retrieves the converted minimum target position for the motor.
     *
     * @return The converted minimum target position as a double.
     */
    public double getMinimumTargetPosition() {
        return minimumTargetPosition;
    }

    /**
     * Retrieves the current position of the motor's encoder.
     *
     * @return the position of the encoder in degrees.
     */
    public double getEncoderPosition() {
        return useAbsoluteEncoder ? absoluteEncoder.getPosition() : relativeEncoder.getPosition();
    }

    /**
     * Retrieves the current velocity of the motor's encoder.
     *
     * @return the velocity of the encoder in degrees per minute.
     */
    public double getEncoderVelocity() {
        return useAbsoluteEncoder ? absoluteEncoder.getVelocity() : relativeEncoder.getVelocity();
    }

    /**
     * Retrieves the voltage applied to the motor.
     *
     * @return the voltage applied to the motor as a Voltage object.
     */
    public Voltage getVoltage() {
        return Units.Volts.of(motor.getBusVoltage() * motor.getAppliedOutput());
    }

    /**
     * Stops the motor
     */
    public void stop() {
        motor.stopMotor();
    }
}
