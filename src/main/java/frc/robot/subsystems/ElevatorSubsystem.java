package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.config.ElevatorSubsystemConfig;
import frc.robot.config.SubsystemsConfig;
import frc.robot.devices.DualElevatorMotor;
import frc.robot.devices.Motor;

@Logged
public class ElevatorSubsystem extends AbstractSetAndSeekSubsystem<ElevatorSubsystemConfig> {

    private ElevatorFeedforward feedforward = new ElevatorFeedforward(0.28209 / 100.0, 0.57002, 28.099 / 100.0, 5.8949 / 100.0);

    public ElevatorSubsystem(SubsystemsConfig subsystemsConfig) {
        super(subsystemsConfig.elevatorSubsystem);
        if (checkDisabled()) {
            return;
        }
    }

    @Override
    protected double calculateVoltageWithVelocities(double currentVelocity, double nextVelocity) {
        return feedforward.calculateWithVelocities(currentVelocity, nextVelocity);
    }

    @Override
    protected double calculateVoltage(double velocity) {
        return feedforward.calculate(velocity);
    }

    @Override
    protected Motor createMotor() {
        return new DualElevatorMotor(config.leftMotorCanId, config.rightMotorCanId, minimumSetPoint, maximumSetPoint);
    }

    /**
     * Logs subsystem motor activity for SysId
     *
     * @param log used to collect data
     * @return void
     */
    protected void logActivity(SysIdRoutineLog routineLog) {
        routineLog.motor("elevator").voltage(motor.getVoltage())
                .linearPosition(Units.Meters.of(motor.getEncoderPosition()))
                .linearVelocity(Units.MetersPerSecond.of(motor.getEncoderVelocity()));
    }
}
