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
import frc.robot.commands.TestLoggerCommand;
import frc.robot.commands.manipulator.shoulder.ShoulderCommand;
import frc.robot.config.ShoulderSubsystemConfig;
import frc.robot.config.SubsystemsConfig;
import frc.robot.devices.PositionalMotor;

/**
 *
 */
@Logged
public class ShoulderSubsystem extends ObotSubsystemBase<ShoulderSubsystemConfig> {

    public BooleanSupplier         clearToSpin                   = () -> {
                                                                     return true;
                                                                 };

    // kS, kG, kV, kA
    // TODO: do we need these for loaded intakes?
    // ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.0, 4.0, 0.0);
    SimpleMotorFeedforward         feedforward                   = new SimpleMotorFeedforward(0.25124, 5.4867, 0.67516);

    private final double           minimumEncoderPositionDegrees = -170.00;

    private final double           maximumEncoderPositionDegrees = 170.00;

    private final double           kDt                           = 0.2;

    private PositionalMotor        shoulderMotor;

    // TODO: max speed/accel?
    private final TrapezoidProfile profile                       = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(0.1, 0.05));

    private TrapezoidProfile.State goal                          = new TrapezoidProfile.State();

    private TrapezoidProfile.State setpoint                      = new TrapezoidProfile.State(-90.0, 0.0);

    /**
     * Construct a new Shoulder Subsustem
     */
    public ShoulderSubsystem(SubsystemsConfig subsystemsConfig) {
        super(subsystemsConfig.shoulderSubsystem);
        if (checkDisabled()) {
            return;
        }

        shoulderMotor = new PositionalMotor(config.motorCanId, minimumEncoderPositionDegrees,
                maximumEncoderPositionDegrees);
    }

    @Override
    public void periodic() {
        if (checkDisabled()) {
            return;
        }

        atTarget();

        log.dashboardVerbose("setpointPosition", setpoint.position);
        log.dashboardVerbose("goalPosition", goal.position);
        log.dashboardVerbose("actualPosition", shoulderMotor.getEncoderPosition());
    }

    /**
     * Sets the target angle for the shoulder to seek
     *
     * @param degrees that the shoulder needs turn to
     * @return void
     */
    public void setTarget(double degrees) {
        if (checkDisabled()) {
            return;
        }

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
     * Seeks the target angle for the shoulder
     *
     * @return void
     */
    public void seekTarget() {
        if (checkDisabled()) {
            return;
        }

        setpoint = profile.calculate(kDt, setpoint, goal);

        var calculatedVoltage = feedforward.calculateWithVelocities(shoulderMotor.getEncoderVelocity(),
                setpoint.velocity);
        log.dashboardVerbose("calculatedVoltage", calculatedVoltage);

        shoulderMotor.setVoltage(calculatedVoltage);
    }

    /**
     * Sets a fixed command
     *
     * @return void
     */
    public void setConstant(double volts) {
        if (checkDisabled()) {
            return;
        }

        shoulderMotor.setVoltage(volts);
    }

    /**
     * Hold the shoulder at the current angle
     *
     * @return void
     */
    public void hold() {
        if (checkDisabled()) {
            return;
        }

        var calculatedVoltage = feedforward.calculate(0.0);
        log.verbose("Calculated Voltage:" + calculatedVoltage);

        shoulderMotor.setVoltage(calculatedVoltage);
    }

    /**
     * Hold the shoulder at the current angle
     *
     * @return void
     */
    public void stop() {
        if (checkDisabled()) {
            return;
        }

        shoulderMotor.setVoltage(0.0);
    }

    /**
     * Determines if the shoulder is at the target angle
     *
     * @return True if the shoulder is at the target angle
     */
    public boolean atTarget() {
        if (checkDisabled()) {
            return false;
        }

        var degreesDifference   = setpoint.position - goal.position;
        var marginOfError       = Math.abs(degreesDifference);

        var withinMarginOfError = marginOfError < 1.0;
        log.dashboardVerbose("marginOfError", marginOfError);

        return withinMarginOfError;
    }

    /**
     * Returns true if the shoulder is at an angle where it can be stowed
     *
     * @return True if the shoulder is at an angle where it can be stowed
     */
    public boolean isStowed() {
        if (checkDisabled()) {
            return false;
        }

        double degrees = setpoint.position;
        return (degrees > -91.0) && (degrees < -89.0);
    }

    /**
     * Creates a new Shoulder command to move to the desired angle
     *
     * @param degrees that the shoulder needs turn to
     * @return Command to move the shoulder
     */
    public Command shoulderCommand(double degrees) {
        if (checkDisabled()) {
            return new TestLoggerCommand("shoulderCommand method not called");
        }

        return new ShoulderCommand(this, degrees);
    }

    public Command shoulderConstant(double volts) {
        if (checkDisabled()) {
            return new TestLoggerCommand("shoulderConstant method not called");
        }

        return Commands.startEnd(() -> this.setConstant(volts), () -> this.stop());
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
        if (checkDisabled()) {
            return new TestLoggerCommand("generateSysIdCommand method not called");
        }

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
        routineLog.motor("shoulder").voltage(shoulderMotor.getVoltage())
                .angularPosition(Units.Degrees.of(shoulderMotor.getEncoderPosition()))
                .angularVelocity(Units.DegreesPerSecond.of(shoulderMotor.getEncoderVelocity()));
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
