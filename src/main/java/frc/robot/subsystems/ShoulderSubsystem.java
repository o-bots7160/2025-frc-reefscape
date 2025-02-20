package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.commands.manipulator.ShoulderCommand;
import frc.robot.devices.PositionalMotor;

/**
 *
 */
@Logged
public class ShoulderSubsystem extends ObotSubsystemBase {

    public BooleanSupplier         clearToSpin                   = () -> {
                                                                     return true;
                                                                 };

    // kS, kG, kV, kA
    // TODO: do we need these for loaded intakes?
    // ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.0, 4.0, 0.0);
    SimpleMotorFeedforward         feedforward                   = new SimpleMotorFeedforward(0.0, 1.0, 0.0);

    private final double           minimumEncoderPositionDegrees = -270.00;

    private final double           maximumEncoderPositionDegrees = 270.00;

    private final double           kDt                           = 0.02;

    private PositionalMotor        shoulderMotor;

    // TODO: max speed/accel?
    private final TrapezoidProfile profile                       = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(0.5, 0.10));

    private TrapezoidProfile.State goal                          = new TrapezoidProfile.State();

    private TrapezoidProfile.State setpoint                      = new TrapezoidProfile.State(-90.0, 0.0);

    /**
     * Construct a new Shoulder Subsustem
     */
    public ShoulderSubsystem() {
        shoulderMotor = new PositionalMotor(53, minimumEncoderPositionDegrees, maximumEncoderPositionDegrees);
    }

    @Override
    public void periodic() {

        var encoder     = shoulderMotor.getAbsoluteEncoder();

        var oldSetpoint = new TrapezoidProfile.State(encoder.getPosition(), encoder.getVelocity());
        setpoint = profile.calculate(kDt, setpoint, goal);

        var calculatedVoltage = feedforward.calculateWithVelocities(oldSetpoint.velocity, setpoint.velocity);
        log.verbose("Calculated Voltage:" + calculatedVoltage);

        shoulderMotor.setVoltage(calculatedVoltage);

        atTarget();

        log.dashboardVerbose("position", setpoint.position);
        log.dashboardVerbose("rel", encoder.getPosition());
        log.dashboardVerbose("target", goal.position);
        log.dashboardVerbose("Voltage", calculatedVoltage);

    }

    @Override
    public void simulationPeriodic() {
    }

    /**
     * Sets the target angle for the shoulder to seek
     *
     * @param degrees that the shoulder needs turn to
     * @return void
     */
    public void setTarget(double degrees) {
        // Checking degrees against limits
        if (degrees > maximumEncoderPositionDegrees) {
            degrees = maximumEncoderPositionDegrees;
        }

        if (degrees < minimumEncoderPositionDegrees) {
            degrees = minimumEncoderPositionDegrees;
        }

        // Update the goal to the degrees with limits applied
        goal = new TrapezoidProfile.State(degrees, 0.0);
    }

    /**
     * Determines if the shoulder is at the target angle
     *
     * @return True if the shoulder is at the target angle
     */
    public boolean atTarget() {
        var degreesDifference   = setpoint.position - goal.position;
        var marginOfError       = Math.abs(degreesDifference);

        var withinMarginOfError = marginOfError < 1.0;
        log.verbose(
                String.format("atTarget: Degrees Difference: %.2f, Margin of Error: %.2f, Within Margin of Error: %b",
                        degreesDifference, marginOfError, withinMarginOfError));

        return withinMarginOfError;
    }

    /**
     * Returns true if the shoulder is at an angle where it can be stowed
     *
     * @return True if the shoulder is at an angle where it can be stowed
     */
    public boolean isStowed() {
        double degrees = Math.toDegrees(setpoint.position); // Could do this in radians... but do we really want to?
        return (degrees > -91.0) && (degrees < -89.0);
    }

    /**
     * Creates a new Shoulder command to move to the desired angle
     *
     * @param degrees that the shoulder needs turn to
     * @return Command to move the shoulder
     */
    public Command shoulderCommand(double degrees) {
        log.debug("Creating shoulder command");
        return new ShoulderCommand(this, degrees);
    }

    /**
     * Creates a command that can be mapped to a button or other trigger. Delays can
     * be set to customize the length of each part of the SysId Routine
     *
     * @param delay          - seconds between each portion to allow motors to spin
     *                       down, etc...
     * @param quasiTimeout   - seconds to run the Quasistatic routines, so robot
     *                       doesn't get too far
     * @param dynamicTimeout - seconds to run the Dynamic routines, 2-3 secs should
     *                       be enough
     * @return A command that can be mapped to a button or other trigger
     */
    public Command generateSysIdCommand(double delay, double quasiTimeout, double dynamicTimeout) {
        SysIdRoutine routine = setSysIdRoutine(new Config());

        return routine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(quasiTimeout)
                .andThen(Commands.waitSeconds(delay))
                .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(quasiTimeout))
                .andThen(Commands.waitSeconds(delay))
                .andThen(routine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(dynamicTimeout))
                .andThen(Commands.waitSeconds(delay))
                .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(dynamicTimeout));
    }

    /**
     * Sets motor voltages
     *
     * @param voltage that the elevator needs to go to
     * @return void
     */
    private void setVoltage(Voltage voltage) {
        shoulderMotor.setVoltage(voltage);
    }

    /**
     * Logs elevator motor activity for SysId
     *
     * @param log used to collect data
     * @return void
     */
    private void logActivity(SysIdRoutineLog routineLog) {
        var encoder = shoulderMotor.getAbsoluteEncoder();
        routineLog.motor("shoulder").voltage(shoulderMotor.getVoltage())
                .angularPosition(Units.Radians.of(encoder.getPosition()))
                .angularVelocity(Units.RadiansPerSecond.of(encoder.getVelocity()));
    }

    /**
     * Creates a SysIdRoutine for this subsystem
     *
     * @param config - The Sys Id routine runner
     * @return A command that can be mapped to a button or other trigger
     */
    private SysIdRoutine setSysIdRoutine(Config config) {
        return new SysIdRoutine(config, new SysIdRoutine.Mechanism((volts) -> this.setVoltage(volts),
                (routineLog) -> this.logActivity(routineLog), this));
    }
}
