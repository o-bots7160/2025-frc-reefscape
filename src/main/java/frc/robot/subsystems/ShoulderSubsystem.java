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
 * Shoulder (rotational joint) subsystem implementation built on {@link AbstractSetAndSeekSubsystem}.
 **/
@Logged
public class ShoulderSubsystem extends AbstractSetAndSeekSubsystem<ShoulderSubsystemConfig> {

    // Feedforward model constants (ks, kv, ka). Using SimpleMotorFeedforward instance for future-proofing
    // but avoiding deprecated accel API by doing manual calculation with these constants.
    private static final double kS          = 0.13291;

    private static final double kV          = 11.52;

    private static final double kA          = 1.4547;

    SimpleMotorFeedforward      feedforward = new SimpleMotorFeedforward(kS, kV, kA);

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
        // Use configured stowed position with tolerance instead of hard-coded band
        double position = motor.getEncoderPosition();
        return Math.abs(position - config.stowedPosition) <= config.setPointTolerance;
    }

    @Override
    protected double calculateVoltageWithVelocities(double currentVelocity, double nextVelocity) {
        // Feedforward (dynamic) calculation path invoked during active profile following.
        // Δv threshold below which we force accel = 0 (suppresses noise-induced acceleration artifacts)
        final double accelDeadbandRadPerSec = Math.toRadians(0.05);

        double       prevRadPerSec          = Math.toRadians(currentVelocity);
        double       nextRadPerSec          = Math.toRadians(nextVelocity);

        if (Math.abs(nextRadPerSec - prevRadPerSec) < accelDeadbandRadPerSec) {
            // Force identical so internal accel = 0
            prevRadPerSec = nextRadPerSec;
        }

        // Library variant in this WPILib version accepts only (prevVel, nextVel) and uses the nominal loop period internally.
        double voltage = feedforward.calculateWithVelocities(prevRadPerSec, nextRadPerSec);
        log.dashboardVerbose("ffTotalVoltage", voltage);
        return voltage;
    }

    @Override
    protected double calculateVoltage(double velocity) {
        // Hold / constant-velocity feedforward (acceleration assumed zero). Invoked when the base class decides we are
        // at target and should maintain position rather than advancing the motion profile.
        double radiansPerSecond = Math.toRadians(velocity);
        double baseVoltage      = feedforward.calculate(radiansPerSecond); // kS + kV * v
        log.dashboardVerbose("holdVelDegPerSec", velocity);
        log.dashboardVerbose("holdVelRadPerSec", radiansPerSecond);
        log.dashboardVerbose("holdBaseVoltage", baseVoltage);
        return baseVoltage; // No accel term when velocity target is steady.
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
