package frc.robot.devices;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.helpers.Logger;

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
public class PositionalMotor {

    private SparkMax motor;

    private Logger   log              = Logger.getInstance(this.getClass());

    private double   conversionFactor = 2.0 * Math.PI;

    private double   convertedMaximumTargetPosition;

    private double   convertedMinimumTargetPosition;

    /**
     * Constructs a PositionalMotor with the specified device ID and target position
     * limits.
     *
     * @param deviceId              The ID of the motor device.
     * @param minimumTargetPosition The minimum target position for the motor.
     * @param maximumTargetPosition The maximum target position for the motor.
     */
    public PositionalMotor(int deviceId, double minimumTargetPosition, double maximumTargetPosition) {
        // apply conversions
        convertedMaximumTargetPosition = maximumTargetPosition * conversionFactor;
        convertedMinimumTargetPosition = minimumTargetPosition * conversionFactor;

        log.verbose("Converted maximum target position: " + maximumTargetPosition + " to: "
                + convertedMaximumTargetPosition);
        log.verbose("Converted minimum target position: " + minimumTargetPosition + " to: "
                + convertedMinimumTargetPosition);

        motor = new SparkMax(deviceId, MotorType.kBrushless);
        log.verbose("Configuring brushless SparkMax motor with device ID " + deviceId);

        SparkMaxConfig config = new SparkMaxConfig();

        // Basic config
        config.inverted(false).voltageCompensation(12.0).idleMode(IdleMode.kBrake);

        // Absolute encoder config
        config.absoluteEncoder.inverted(false)
                // Setting conversion factors
                .positionConversionFactor(conversionFactor).velocityConversionFactor(conversionFactor)
                // center output range: -0.5 to 0.5 rather than 0.0 to 1.0
                .zeroCentered(true)
                // TODO: Should this be calibrated straight down?
                .zeroOffset(0.0)
                // Apparently required... Whats it do? Nobody knows.
                .setSparkMaxDataPortConfig();

        // Soft limit config
        config.softLimit
                // Setting the forward soft limit with the conversion factor applied
                .forwardSoftLimit(convertedMaximumTargetPosition).forwardSoftLimitEnabled(false)
                // setting the reverse soft limit with the conversion factor applied
                .reverseSoftLimit(convertedMinimumTargetPosition).reverseSoftLimitEnabled(false);

        // load the configuration into the motor
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

    public AbsoluteEncoder getAbsoluteEncoder() {
        return motor.getAbsoluteEncoder();
    }

    public RelativeEncoder getRelativeEncoder() {
        return motor.getEncoder();
    }

    public double getConvertedMaximumTargetPosition() {
        return convertedMaximumTargetPosition;
    }

    public double getConvertedMinimumTargetPosition() {
        return convertedMinimumTargetPosition;
    }

}
