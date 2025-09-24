package frc.robot.devices;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;

public class ShoulderMotor extends AbstractMotor {

    private RelativeEncoder encoder;

    public ShoulderMotor(int deviceId, double minimumTargetPosition, double maximumTargetPosition, double conversionFactor) {
        super("ShoulderMotor", deviceId, minimumTargetPosition, maximumTargetPosition, conversionFactor);
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
        config.inverted(true).idleMode(IdleMode.kBrake);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.encoder
                .positionConversionFactor(conversionFactor)
                // Dividing by 60 as velocity is per minute and we need per second
                .velocityConversionFactor(conversionFactor / 60.0);

        config.softLimit
                .forwardSoftLimit(maximumTargetPosition).forwardSoftLimitEnabled(false)
                .reverseSoftLimit(minimumTargetPosition).reverseSoftLimitEnabled(false);
        return config;
    }
}
