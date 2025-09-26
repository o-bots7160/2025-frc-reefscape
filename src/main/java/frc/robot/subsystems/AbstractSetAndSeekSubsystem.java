package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.config.AbstractSetAndSeekSubsystemConfig;
import frc.robot.devices.Motor;
import frc.robot.motion.TrapezoidalMotionManager;

/**
 * Abstract base class for single-motor mechanisms that move to target positions using motion profiling.
 * <p>
 * This class provides a complete framework for controlling mechanisms like elevators, arms, wrists, shooters, or any single-axis motorized system
 * that needs precise positioning with smooth motion.
 * <p>
 * <b>Core Functionality:</b>
 * <ul>
 * <li><b>Motion Profiling:</b> Generates trapezoidal velocity profiles for smooth acceleration/deceleration</li>
 * <li><b>Position Control:</b> PID feedback with optional feedforward compensation for accurate positioning</li>
 * <li><b>Safety Systems:</b> Software limits, tolerance checking, and predefined safe positions</li>
 * <li><b>State Management:</b> Tracks stowed/cleared positions for collision avoidance</li>
 * <li><b>Command Interface:</b> Pre-built commands for seeking positions, stopping, and characterization</li>
 * </ul>
 * <p>
 * <b>Usage Pattern:</b> Subclasses implement {@link #createMotor()}, {@link #calculateVoltage(double)}, and {@link #logActivity(SysIdRoutineLog)}.
 * The base class handles motion profiling, control loops, safety limits, dashboard integration, and provides ready-to-use commands for common
 * operations.
 * <p>
 * <b>Key Methods:</b>
 * <ul>
 * <li>{@link #setTarget(double)} - Set desired position with motion profiling</li>
 * <li>{@link #seekTarget()} - Execute one control loop iteration</li>
 * <li>{@link #atTarget()} - Check if mechanism has reached target within tolerance</li>
 * <li>{@link #isStowed()}/{@link #isClear()} - Safety position queries</li>
 * <li>{@link #seekTarget(Supplier)} - Command factory for position seeking</li>
 * </ul>
 * 
 * @param <TConfig> concrete config type containing tuning constants, limits, and mechanism-specific parameters
 */
public abstract class AbstractSetAndSeekSubsystem<TConfig extends AbstractSetAndSeekSubsystemConfig> extends AbstractSubsystem<TConfig> {

    // Optional delay gate for verbose outputs
    protected int                                       verboseDelay     = 0;

    // Position considered "cleared" for interferences
    protected double                                    clearedPosition;

    // Position considered safely stowed
    protected double                                    stowedPosition;

    // Motion manager encapsulates trapezoidal profile state machine
    protected frc.robot.motion.TrapezoidalMotionManager motionManager;

    // Upper software limit (retained for semantic clarity / clamps via manager construction)
    protected double                                    maximumSetPoint;

    // Lower software limit
    protected double                                    minimumSetPoint;

    // Allowed position error at target
    protected double                                    setPointTolerance;

    // Reused as velocity tolerance when at target
    protected double                                    stoppingTolerance;

    // Optional label for child sendable
    protected String                                    motorName;

    // Concrete motor implementation created by subclass
    protected Motor                                     motor;

    /** Convenience getters after manager advance */
    protected State                                     nextState;             // mirror of manager.getNextState()

    // Last profiled acceleration (for telemetry)
    // Cached telemetry mirrors from motion manager
    private double                                      lastAcceleration = 0.0;

    private double                                      lastMeasuredPos  = 0.0;

    private double                                      lastMeasuredVel  = 0.0;

    /**
     * Constructs the subsystem, loading configuration limits and optionally creating the motor if enabled. Seeds profile state at zero.
     * 
     * @param config configuration object providing limits & constants
     */
    protected AbstractSetAndSeekSubsystem(TConfig config) {
        super(config);
        // Copy static configuration values
        minimumSetPoint   = config.minimumSetPoint;
        maximumSetPoint   = config.maximumSetPoint;
        setPointTolerance = config.setPointTolerance;
        clearedPosition   = config.clearedPosition;
        stoppingTolerance = config.stoppingTolerance;
        stowedPosition    = config.stowedPosition;
        // Start profile state
        motionManager     = new TrapezoidalMotionManager(minimumSetPoint, maximumSetPoint, setPointTolerance, stoppingTolerance,
                config.maximumVelocity, config.maximumAcceleration);
        nextState         = motionManager.getNextState();
        // Create motor only if subsystem is enabled (allows graceful disabling in sim or tests)
        if (isEnabled()) {
            motor = createMotor();
            addChild("Motor", motor);
            SmartDashboard.putData(className + "/Motor", motor);
            // Register a default command that will continue managing motion (seeking/holding)
            if (config.enableDefaultCommand) {
                setDefaultCommand(createDefaultCommand());
            }
        }
    }

    /**
     * Standard periodic hook. Currently just early-exits if disabled. Override in subclasses if additional periodic work is needed.
     */
    @Override
    public void periodic() {
        if (checkDisabled()) {
            // Skip all logic when disabled
            return;
        }
    }

    /**
     * Set a new target position (in mechanism units) for the profiling system.
     * <p>
     * Clamps to min/max software limits and seeds the profile from the CURRENT measured state (position & velocity) so motion remains smooth.
     * 
     * @param setPoint desired end position (will finish at rest)
     */
    public void setTarget(double setPoint) {
        if (checkDisabled()) {
            return;
        }
        double measuredPos = motor.getEncoderPosition();
        double measuredVel = motor.getEncoderVelocity();
        motionManager.setTarget(setPoint, measuredPos, measuredVel);
        nextState = motionManager.getNextState();
    }

    /**
     * Request a velocity change by converting it into a setpoint that allows the profile to accelerate/decelerate to the new velocity. Uses kinematic
     * distance = (v1^2 - v2^2)/(2a) to compute the displacement required to reach the new velocity, then sets a target offset from current position.
     * 
     * @param velocity desired target velocity (units per second)
     */
    public void setTargetVelocity(double velocity) {
        if (checkDisabled()) {
            return;
        }
        // Distance needed to change from current profiled velocity to desired velocity at max accel
        double measuredPos       = motor.getEncoderPosition();
        double currentProfileVel = motionManager.getNextState().velocity;
        motionManager.setTargetVelocity(velocity, measuredPos, currentProfileVel);
        nextState = motionManager.getNextState();
    }

    /**
     * Gracefully interrupt current motion. Creates a new goal that allows the mechanism to decelerate without overshooting the original goal. Used by
     * commands when they are interrupted / canceled.
     */
    public void interrupt() {
        if (checkDisabled()) {
            return;
        }
        // Capture current motion state
        motionManager.interrupt(motor.getEncoderPosition(), motor.getEncoderVelocity());
        nextState = motionManager.getNextState();
    }

    /**
     * @return current measured encoder position (mechanism units)
     */
    public double getCurrentPosition() {
        return motor.getEncoderPosition();
    }

    /**
     * @return current measured encoder velocity (units per second)
     */
    public double getCurrentVelocity() {
        return motor.getEncoderVelocity();
    }

    /**
     * Advance the trapezoid profile one loop, compute control voltage (PID + FF or subclass feedforward), and apply it. Locks into a zero-velocity
     * hold once target is achieved and resets PID accumulators.
     * 
     * @return applied control voltage (clamped to [-12, 12])
     */
    public double seekTarget() {
        if (checkDisabled()) {
            return 0.0;
        }
        // Store previous profiled state (for acceleration / feedforward)
        double                                   measuredPos = motor.getEncoderPosition();
        double                                   measuredVel = motor.getEncoderVelocity();
        TrapezoidalMotionManager.AdvanceSnapshot snapshot    = motionManager.advance(measuredPos, measuredVel, kDt);
        lastMeasuredPos  = measuredPos;
        lastMeasuredVel  = measuredVel;
        nextState        = motionManager.getNextState();
        lastAcceleration = motionManager.getLastAcceleration();
        if (snapshot.overshootReanchored) {
            log.warning(className + ": Overshoot detected; profile re-anchored for reversal.");
        }
        State previousState = snapshot.previousProfiled;
        State measuredState = snapshot.measured;
        // Verbose logging of profiling internals
        log.dashboardVerbose("profiledVelocity", nextState.velocity);
        log.dashboardVerbose("profiledPosition", nextState.position);
        log.dashboardVerbose("measuredVelocity", measuredState.velocity);
        log.dashboardVerbose("measuredPosition", measuredState.position);
        log.dashboardVerbose("acceleration", lastAcceleration);
        double voltage = computeControlVoltage(previousState, measuredState);
        setVoltage(voltage);
        // If we have arrived, lock state to prevent tiny residual profile motion
        if (atTarget()) {
            motionManager.lockAtGoal();
            nextState = motionManager.getNextState();
        }
        return voltage;
    }

    /**
     * Directly set the motor output voltage (bypassing profile logic). Used internally and for manual control commands.
     * 
     * @param voltage desired voltage (-12 to +12 typically)
     */
    public void setVoltage(double voltage) {
        if (checkDisabled()) {
            return;
        }
        motor.setVoltage(voltage);
    }

    /**
     * Determine if mechanism is within the stowed region.
     * 
     * @return true if encoder position is between 0 and stowedPosition
     */
    public boolean isStowed() {
        if (checkDisabled()) {
            return false;
        }
        double currentPosition = motor.getEncoderPosition();
        return (currentPosition >= 0.0) && (currentPosition < stowedPosition);
    }

    /**
     * Command the mechanism to move toward the predefined stow position.
     */
    public void setStow() {
        if (checkDisabled()) {
            return;
        }
        setTarget(stowedPosition);
    }

    /**
     * @return true if mechanism is past the cleared position threshold.
     */
    public boolean isClear() {
        if (checkDisabled()) {
            return true;
        }
        return motor.getEncoderPosition() > clearedPosition;
    }

    /**
     * Set target to the cleared position mark.
     */
    public void setClear() {
        if (checkDisabled()) {
            return;
        }
        setTarget(clearedPosition);
    }

    /**
     * Apply a holding voltage computed by subclass feedforward (with zero desired velocity). Useful for mechanisms fighting gravity.
     */
    public void hold() {
        if (checkDisabled()) {
            return;
        }
        double calculatedVoltage = calculateVoltage(0.0);
        log.verbose("Calculated Voltage:" + calculatedVoltage);
        setVoltage(calculatedVoltage);
    }

    /**
     * Immediately stop by converting the current position into a zero-velocity goal and setting motor voltage to zero. (No abrupt reversal behavior.)
     */
    public void stop() {
        if (checkDisabled()) {
            return;
        }
        double pos = motor.getEncoderPosition();
        motionManager.setTarget(pos, pos, motor.getEncoderVelocity());
        nextState = motionManager.getNextState();
        setVoltage(0.0);
    }

    /**
     * True when both position error magnitude and measured velocity are within configured tolerances (indicates stable arrival).
     * 
     * @return whether mechanism is at target and sufficiently stationary
     */
    public boolean atTarget() {
        if (checkDisabled()) {
            return true;
        }
        double  measuredPos = motor.getEncoderPosition();
        double  measuredVel = motor.getEncoderVelocity();
        boolean at          = motionManager.atTarget(measuredPos, measuredVel);
        log.dashboardVerbose("posError", Math.abs(measuredPos - motionManager.getGoalState().position));
        log.dashboardVerbose("withinPos", Math.abs(measuredPos - motionManager.getGoalState().position) < setPointTolerance);
        log.dashboardVerbose("withinVel", Math.abs(measuredVel) < stoppingTolerance);
        return at;
    }

    /**
     * Create a command that seeks the supplied position setpoint using motion profiling until {@link #atTarget()} returns true. Interruptions trigger
     * a smooth deceleration.
     * 
     * @param position supplier providing desired target position each schedule (initialization only reads it)
     * @return constructed command
     */
    public Command seekTarget(Supplier<Double> position) {
        return new FunctionalCommand(
                () -> setTarget(position.get()),
                this::seekTarget,
                interrupted -> {
                    if (interrupted) {
                        // Schedule smooth decel
                        interrupt();
                    } else {
                        stop();
                    }
                },
                this::atTarget, this);
    }

    /**
     * Build a SysId characterization command sequence (quasi-static forward/reverse and dynamic forward/reverse) with delays between phases.
     * 
     * @param delay          seconds delay between routine phases
     * @param quasiTimeout   timeout for each quasistatic direction
     * @param dynamicTimeout timeout for each dynamic direction
     * @return composite command sequence
     */
    public Command generateSysIdCommand(double delay, double quasiTimeout, double dynamicTimeout) {
        if (checkDisabled()) {
            return new InstantCommand(() -> log.verbose("generateSysIdCommand method not called"));
        }
        Config                 sysIdRoutineConfig = new Config();
        SysIdRoutine.Mechanism sysIdMechanism     = new SysIdRoutine.Mechanism((v) -> setVoltage(v.baseUnitMagnitude()), this::logActivity, this);
        SysIdRoutine           routine            = new SysIdRoutine(sysIdRoutineConfig, sysIdMechanism);
        return routine
                .quasistatic(SysIdRoutine.Direction.kForward)
                .withTimeout(quasiTimeout)
                .andThen(Commands.waitSeconds(delay))
                .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(quasiTimeout))
                .andThen(Commands.waitSeconds(delay))
                .andThen(routine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(dynamicTimeout))
                .andThen(Commands.waitSeconds(delay))
                .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(dynamicTimeout));
    }

    /**
     * Add telemetry properties to WPILib sendable infrastructure for dashboards.
     * 
     * @param builder sendable builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        if (isDisabled()) {
            return;
        }
        builder.addDoubleProperty("setPoint", () -> motionManager.getGoalState().position, this::setTarget);
        builder.addDoubleProperty("currentPosition", () -> motor.getEncoderPosition(), null);
        builder.addDoubleProperty("currentVelocity", () -> motor.getEncoderVelocity(), null);
        builder.addDoubleProperty("stowedPosition", () -> stowedPosition, null);
        builder.addDoubleProperty("clearedPosition", () -> clearedPosition, null);
        builder.addDoubleProperty("profiledPosition", () -> nextState.position, null);
        builder.addDoubleProperty("profiledVelocity", () -> nextState.velocity, null);
        builder.addDoubleProperty("measuredPosition", () -> lastMeasuredPos, null);
        builder.addDoubleProperty("measuredVelocity", () -> lastMeasuredVel, null);
        builder.addDoubleProperty("acceleration", () -> lastAcceleration, null);
    }

    /**
     * Default command that continuously manages the motion profile when nothing else is commanding the subsystem. Behavior: - If the mechanism has
     * not yet reached its goal (including an interrupted decel goal), continue calling seekTarget() to advance the profile. - Once at target, apply a
     * holding feedforward (e.g. gravity compensation) instead of re-running the profile calculation. This prevents the motor from being left with the
     * last arbitrary voltage of an interrupted command.
     */
    protected Command createDefaultCommand() {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    if (checkDisabled()) {
                        return;
                    }
                    if (atTarget()) {
                        hold(); // maintain position (gravity/static friction compensation)
                    } else {
                        seekTarget(); // continue decel or move toward goal
                    }
                },
                interrupted -> {
                    // No special action; if another command interrupted this one it will take over control.
                },
                () -> false,
                this);
    }

    /**
     * Subclasses must create and configure the underlying motor (controller, inversion, current limiting, etc.).
     * 
     * @return initialized Motor instance
     */
    protected abstract Motor createMotor();

    /**
     * Subclass hook to compute a feedforward voltage given current and next profile velocities. (Legacy API retained for mechanisms not using PID
     * internal loop.)
     * 
     * @param currentVelocity measured/current velocity
     * @param nextVelocity    next profiled velocity
     * @return feedforward voltage
     */
    protected abstract double calculateVoltageWithVelocities(double currentVelocity, double nextVelocity);

    /**
     * Subclass hook to compute a feedforward voltage for an arbitrary velocity target (e.g. kS + kV*v + kA*a + kG form).
     * 
     * @param velocity desired velocity
     * @return feedforward voltage
     */
    protected abstract double calculateVoltage(double velocity);

    /**
     * Subclass logging hook for SysId characterization (record mechanism-specific data into provided routine log).
     * 
     * @param routineLog sysid log object
     */
    protected abstract void logActivity(SysIdRoutineLog routineLog);

    // Compute control voltage (feedforward only – custom PID removed)
    private double computeControlVoltage(State previousProfiledState, State measuredState) {
        double voltage = calculateVoltageWithVelocities(previousProfiledState.velocity, motionManager.getNextState().velocity);
        if (voltage > 12.0)
            voltage = 12.0;
        if (voltage < -12.0)
            voltage = -12.0;
        log.dashboardVerbose("ffVoltage", voltage);
        return voltage;
    }
}
