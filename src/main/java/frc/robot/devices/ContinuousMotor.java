package frc.robot.devices;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.helpers.Logger;

public class ContinuousMotor {

    protected double          conversionFactor = 360.0;

    protected AbsoluteEncoder encoder;

    protected double          maximumTargetPosition;

    protected double          minimumTargetPosition;

    protected SparkMax        motor;

    protected Logger          log              = Logger.getInstance(this.getClass());

    public ContinuousMotor(int deviceId, double minimumTargetPosition, double maximumTargetPosition) {
        this.minimumTargetPosition = minimumTargetPosition;
        this.maximumTargetPosition = maximumTargetPosition;

        motor                      = new SparkMax(deviceId, MotorType.kBrushless);
        log.verbose("Configuring brushless SparkMax motor with device ID " + deviceId);

        SparkMaxConfig config = new SparkMaxConfig();

        // Basic config
        config.inverted(false).voltageCompensation(12.0).idleMode(IdleMode.kBrake);

        // Absolute encoder config
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        config.absoluteEncoder.inverted(false)
                // Setting conversion factors
                .positionConversionFactor(conversionFactor).velocityConversionFactor(conversionFactor)
                // center output range: -0.5 to 0.5 rather than 0.0 to 1.0
                .zeroCentered(true)
                // TODO: Should this be calibrated straight down?
                .zeroOffset(0.0);

        // Soft limit config
        config.softLimit
                // Setting the forward soft limit with the conversion factor applied
                .forwardSoftLimit(maximumTargetPosition).forwardSoftLimitEnabled(false)
                // setting the reverse soft limit with the conversion factor applied
                .reverseSoftLimit(minimumTargetPosition).reverseSoftLimitEnabled(false);

        // load the configuration into the motor
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Sets the absolute encoder to the motor's encoder
        encoder                    = motor.getAbsoluteEncoder();

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

    public void forward()
    {
        motor.set(0.90);
    }

    public void reverse()
    {
        motor.set(-0.90);
    }

    public void stop()
    {
        motor.stopMotor();
    }

}
