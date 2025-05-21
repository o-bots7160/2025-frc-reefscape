package frc.robot.devices;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class AbstractSimpleMotor extends AbstractMotor {
    protected RelativeEncoder encoder;

    public AbstractSimpleMotor(String name, int deviceId) {
        super(name, deviceId);
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
    protected SparkMaxConfig configureMotor(SparkMaxConfig config) {
        config.inverted(false).idleMode(IdleMode.kBrake);
        return config;
    }
}
