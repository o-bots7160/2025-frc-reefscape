package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import frc.robot.config.ClimberSubsystemConfig;
import frc.robot.config.SubsystemsConfig;
import frc.robot.devices.ClimberMotor;

/**
 *
 */
@Logged
public class ClimberSubsystem extends AbstractSubsystem<ClimberSubsystemConfig> {
    private ClimberMotor motor;

    /**
    *
    */
    public ClimberSubsystem(SubsystemsConfig subsystemsConfig) {
        super(subsystemsConfig.climberSubsystem);
        if (checkDisabled()) {
            return;
        }

        motor = new ClimberMotor(className, config.climberMotorCanId);

        addChild(className + "/Motor", motor);
    }

    public void start(Double speed) {
        if (isDisabled()) {
            return;
        }
        motor.setSpeed(speed);
    }

    public void stop() {
        if (isDisabled()) {
            return;
        }
        motor.setSpeed(0);
    }

    public double getPosition() {
        if (isDisabled()) {
            return 0.0;
        }
        return motor.getEncoderPosition();
    }
}
