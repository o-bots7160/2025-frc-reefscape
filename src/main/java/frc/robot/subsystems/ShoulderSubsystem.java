package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.config.ShoulderSubsystemConfig;
import frc.robot.config.SubsystemsConfig;
import frc.robot.devices.Motor;
import frc.robot.devices.ShoulderMotor;

/**
 *
 */
@Logged
public class ShoulderSubsystem extends AbstractSetAndSeekSubsystem<ShoulderSubsystemConfig> {

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.26235, 10.842, 0.96402);

    /**
     * Construct a new Shoulder Subsustem
     */
    public ShoulderSubsystem(SubsystemsConfig subsystemsConfig) {
        super(subsystemsConfig.shoulderSubsystem);
        if (checkDisabled()) {
            return;
        }

        setVoltage(0.0);
    }

    /**
     * Returns true if the shoulder is at an angle where it can be stowed
     *
     * @return True if the shoulder is at an angle where it can be stowed
     */
    @Override
    public boolean isStowed() {
        if (checkDisabled()) {
            return false;
        }

        double degrees = motor.getEncoderPosition();
        return (degrees > -91.0) && (degrees < -89.0);
    }

    @Override
    protected double calculateVoltageWithVelocities(double currentVelocity, double nextVelocity) {
        return feedforward.calculate(nextVelocity) * 0.0029126;
        // return feedforward.calculateWithVelocities(currentVelocity, nextVelocity);
    }

    @Override
    protected double calculateVoltage(double velocity) {
        return feedforward.calculate(velocity) * 0.0029126;
    }

    @Override
    protected Motor createMotor() {
        return new ShoulderMotor(config.motorCanId, config.minimumSetPoint, config.maximumSetPoint,
                config.conversionFactor);
    }

    /**
     * Logs subsystem motor activity for SysId
     *
     * @param log used to collect data
     * @return void
     */
    @Override
    protected void logActivity(SysIdRoutineLog routineLog) {
        routineLog.motor("shoulder").voltage(motor.getVoltage())
                .angularPosition(Units.Degrees.of(motor.getEncoderPosition()))
                .angularVelocity(Units.DegreesPerSecond.of(motor.getEncoderVelocity()));
    }

}
