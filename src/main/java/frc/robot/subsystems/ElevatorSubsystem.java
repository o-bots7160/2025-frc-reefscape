package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.config.ElevatorSubsystemConfig;
import frc.robot.config.SubsystemsConfig;
import frc.robot.devices.ElevatorMotors;

@Logged
public class ElevatorSubsystem extends SetAndSeekSubsystemBase<ElevatorSubsystemConfig> {

    private ElevatorFeedforward feedforward = new ElevatorFeedforward(0.28209 / 100.0, 0.57002, 28.099 / 100.0, 5.8949 / 100.0);

    public ElevatorSubsystem(SubsystemsConfig subsystemsConfig) {
        super(subsystemsConfig.elevatorSubsystem);
        if (checkDisabled()) {
            return;
        }

        motor = new ElevatorMotors(config.leftMotorCanId, config.rightMotorCanId, minimumSetPoint, maximumSetPoint);
    }

    @Override
    protected double calculateVoltageWithVelocities(double currentVelocity, double nextVelocity) {
        return feedforward.calculateWithVelocities(currentVelocity, nextVelocity);
    }

    @Override
    protected double calculateVoltage(double velocity) {
        return feedforward.calculate(velocity);
    }
}
