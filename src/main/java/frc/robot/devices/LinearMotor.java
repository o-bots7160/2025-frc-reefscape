package frc.robot.devices;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.helpers.Logger;

/**
 * The LinearMotor class represents a motor with positional control
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
public class LinearMotor{

    private double   rotationsToCentimetersConversionFactor = 10.0/(8.0*Math.PI);

    private SparkMax motor;
    private AbsoluteEncoder encoder;

    private Logger   log                                    = Logger.getInstance(this.getClass());

    private double   maximumTargetPosition;

    private double   minimumTargetPosition;

    /**
     * Constructs a LinearMotor with the specified device ID and target position
     * limits.
     *
     * @param deviceId              The ID of the motor device.
     * @param minimumTargetPosition The minimum target position in centimeters for the
     *                              motor.
     * @param maximumTargetPosition The maximum target position in centimeters for the
     *                              motor.
     */

    public LinearMotor(int deviceId, double minimumTargetPosition, double maximumTargetPosition)
    {
        // apply conversions
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
                .positionConversionFactor(rotationsToCentimetersConversionFactor)
                .velocityConversionFactor(rotationsToCentimetersConversionFactor)
                // center output range: -0.5 to 0.5 rather than 0.0 to 1.0
                .zeroCentered(true)
                // TODO: Should this be calibrated straight down?
                .zeroOffset(0.0);

        // Soft limit config
        config.softLimit
                // Setting the forward soft limit with the conversion factor applied
                .forwardSoftLimit(maximumTargetPosition).forwardSoftLimitEnabled(true)
                // setting the reverse soft limit with the conversion factor applied
                .reverseSoftLimit(minimumTargetPosition).reverseSoftLimitEnabled(true);

        // load the configuration into the motor
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = motor.getAbsoluteEncoder();
        // TODO: could we follow a command pattern?
        // https://docs.wpilib.org/en/stable/docs/software/commandbased/pid-subsystems-commands.html
        // Can we create/move PID control on/to motor controller?
        // config.closedLoop
        // .pidf( 0.0001, 0.0, 0.0, 1.0 )
        // .iMaxAccum( 0.5 )
        // .iZone( 0.01 )
        // config.softLimit.feedbackSensor(
        // ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder );

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
