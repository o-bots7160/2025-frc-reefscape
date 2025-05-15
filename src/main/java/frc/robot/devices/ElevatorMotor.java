package frc.robot.devices;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;

public class ElevatorMotor extends MotorBase {

    private RelativeEncoder encoder;

    public ElevatorMotor(String name, int deviceId, double minimumTargetPosition, double maximumTargetPosition, double conversionFactor) {
        super(name, deviceId, minimumTargetPosition, maximumTargetPosition, conversionFactor);
        encoder = motor.getEncoder();
    }

    @Override
    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Position", this::getEncoderPosition, null);
        builder.addDoubleProperty("Velocity", this::getEncoderVelocity, null);
    }

    @Override
    protected SparkMaxConfig configureMotor(SparkMaxConfig config) {
        var isInverted = false;
        config.inverted(isInverted).idleMode(IdleMode.kBrake);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        config.absoluteEncoder.inverted(!isInverted)
                .positionConversionFactor(conversionFactor)
                .velocityConversionFactor(conversionFactor)
                .zeroCentered(true);
        config.absoluteEncoder.zeroOffset(0);

        config.softLimit
                .forwardSoftLimit(maximumTargetPosition).forwardSoftLimitEnabled(false)
                .reverseSoftLimit(minimumTargetPosition).reverseSoftLimitEnabled(false);
        return config;
    }
}
