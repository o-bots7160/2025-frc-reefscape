package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.config.ElevatorSubsystemConfig;
import frc.robot.config.SubsystemsConfig;
import frc.robot.devices.LinearMotor;

@Logged
public class ElevatorSubsystem extends SetAndSeekSubsystemBase<ElevatorSubsystemConfig> {

    private ElevatorFeedforward feedforward = new ElevatorFeedforward(0.28209 / 100.0, 0.57002, 28.099 / 100.0, 5.8949 / 100.0);

    public ElevatorSubsystem(SubsystemsConfig subsystemsConfig) {
        super(subsystemsConfig.elevatorSubsystem);
        if (checkDisabled()) {
            return;
        }

        // Configure motors
        var rightElevatorMotor = new LinearMotor(config.rightMotorCanId, minimumSetPoint, maximumSetPoint);
        var leftElevatorMotor  = new LinearMotor(config.leftMotorCanId, minimumSetPoint, maximumSetPoint);
        motors.put(0, new MotorData(rightElevatorMotor, "rightElevatorMotor"));
        motors.put(1, new MotorData(leftElevatorMotor, "leftElevatorMotor"));
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
