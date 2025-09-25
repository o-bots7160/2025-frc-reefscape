package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    protected int                    verboseDelay      = 0;

    // Position considered "cleared" for interferences
    protected double                 clearedPosition;

    // Position considered safely stowed
    protected double                 stowedPosition;

    // Current motion profile goal (position, vel=0)
    protected TrapezoidProfile.State goalState         = new TrapezoidProfile.State();

    // Shared trapezoid profile object
    protected TrapezoidProfile       profile;

    // Upper software limit
    protected double                 maximumSetPoint;

    // Lower software limit
    protected double                 minimumSetPoint;

    // Allowed position error at target
    protected double                 setPointTolerance;

    // Reused as velocity tolerance when at target
    protected double                 stoppingTolerance;

    // NOTE: spelling preserved for backward compatibility
    protected boolean                isInterupted      = false;

    // Optional label for child sendable
    protected String                 motorName;

    // Concrete motor implementation created by subclass
    protected Motor                  motor;

    /** Internal profiled setpoint from previous calculate() call. */
    protected State                  nextState;

    // Accumulated integral term
    private double                   pidIntegral       = 0.0;

    // For derivative calculation
    private double                   lastPositionError = 0.0;

    // Last profiled acceleration (for telemetry)
    private double                   lastAcceleration  = 0.0;

    // Cached last measured position for sendable
    private double                   lastMeasuredPos   = 0.0;

    // Cached last measured velocity for sendable
    private double                   lastMeasuredVel   = 0.0;

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
        nextState         = new State(0, 0);
        profile           = new TrapezoidProfile(new TrapezoidProfile.Constraints(config.maximumVelocity, config.maximumAcceleration));
        // Create motor only if subsystem is enabled (allows graceful disabling in sim or tests)
        if (isEnabled()) {
            motor = createMotor();
            addChild("Motor", motor);
            SmartDashboard.putData(className + "/Motor", motor);
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
        // Clear interrupted state when a fresh goal is provided
        isInterupted = false;
        // Clamp requested setpoint to allowed limits
        double newSetPoint = Math.max(minimumSetPoint, Math.min(maximumSetPoint, setPoint));
        // Read sensor state to ensure profile starts from reality (not stale state)
        double measuredPos = motor.getEncoderPosition();
        double measuredVel = motor.getEncoderVelocity();
        nextState = new State(measuredPos, measuredVel);
        // Want to end at rest
        goalState = new State(newSetPoint, 0.0);
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
        double currentVel  = nextState.velocity;
        double baseFormula = (Math.pow(currentVel, 2.0) - Math.pow(velocity, 2.0)) / (2.0 * config.maximumAcceleration);
        double setPoint    = currentVel > velocity ? baseFormula : -baseFormula;
        // Use measured position as reference
        setPoint = motor.getEncoderPosition() + setPoint;
        setTarget(setPoint);
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
        double currentPos      = motor.getEncoderPosition();
        double currentVel      = motor.getEncoderVelocity();
        double originalGoal    = goalState.position;
        double brakingDistance = computeBrakingDistance(currentVel);
        // Default to immediate hold if nearly stopped
        double newGoal         = currentPos;
        if (currentVel > 0) {
            // Moving positive direction – limit stopping point to not exceed original goal
            if (currentPos < originalGoal) {
                newGoal = Math.min(currentPos + brakingDistance, originalGoal);
            }
        } else if (currentVel < 0) {
            // Moving negative direction – limit opposite side overshoot
            if (currentPos > originalGoal) {
                newGoal = Math.max(currentPos - brakingDistance, originalGoal);
            }
        }
        goalState    = new State(clampToLimits(newGoal), 0.0);
        nextState    = new State(currentPos, currentVel);
        isInterupted = true;
    }

    /**
     * Returns whether the mechanism is currently in an interrupted deceleration state.
     * 
     * @return true if previously interrupted
     */
    public boolean isInterupted() {
        return isInterupted;
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
        State previousState = nextState;
        // Always use measured state as the start state for the next profile segment
        State measuredState = new State(motor.getEncoderPosition(), motor.getEncoderVelocity());
        lastMeasuredPos  = measuredState.position;
        lastMeasuredVel  = measuredState.velocity;
        // Overshoot safeguard: if velocity sign is driving us farther from the goal (moving away)
        double posError = goalState.position - measuredState.position; // desired - actual
        boolean movingAway = (Math.abs(posError) > setPointTolerance) && (measuredState.velocity != 0.0)
                && (Math.signum(measuredState.velocity) != Math.signum(posError));
        if (movingAway) {
            // Re-anchor the profile at the current position with zero velocity so it can legitimately reverse
            log.warning(className + ": Overshoot detected; re-anchoring profile for reversal. posError=" + posError + ", vel=" + measuredState.velocity);
            measuredState = new State(measuredState.position, 0.0);
            nextState = measuredState; // seed previous for acceleration calc
            // Reset PID terms so we don't carry windup into reversal
            pidIntegral = 0.0;
            lastPositionError = 0.0;
        }
        nextState        = profile.calculate(kDt, measuredState, goalState);
        lastAcceleration = (nextState.velocity - previousState.velocity) / kDt;
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
            goalState         = new State(goalState.position, 0.0);
            nextState         = new State(goalState.position, 0.0);
            // Reset PID terms when complete
            pidIntegral       = 0.0;
            lastPositionError = 0.0;
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
        goalState = new State(pos, 0.0);
        nextState = new State(pos, motor.getEncoderVelocity());
        setVoltage(0.0);
    }

    /**
     * Apply a constant open-loop voltage (for manual testing or characterization support wrappers).
     * 
     * @param volts voltage to apply
     */
    public void setConstant(double volts) {
        if (checkDisabled()) {
            return;
        }
        setVoltage(volts);
    }

    /**
     * Request a graceful stop; maintained for semantic clarity – simply calls {@link #interrupt()}.
     */
    public void requestStop() {
        interrupt();
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
        double  posError    = measuredPos - goalState.position;
        boolean withinPos   = Math.abs(posError) < setPointTolerance;
        // Reuse stoppingTolerance as velocity threshold
        boolean withinVel   = Math.abs(measuredVel) < stoppingTolerance;
        log.dashboardVerbose("posError", Math.abs(posError));
        log.dashboardVerbose("withinPos", withinPos);
        log.dashboardVerbose("withinVel", withinVel);
        return withinPos && withinVel;
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
     * Create a command that alters the profile so the mechanism decelerates to zero velocity from its current motion state.
     * 
     * @return command finishing when motion and position tolerances are met
     */
    public Command seekZeroVelocity() {
        return new FunctionalCommand(
                () -> setTargetVelocity(0.0),
                this::seekTarget,
                interrupted -> {
                    if (interrupted) {
                        interrupt();
                    } else {
                        stop();
                    }
                },
                this::atTarget, this);
    }

    /**
     * Create a command that directly sets voltage from a velocity-based feedforward each init (no profile tracking). Never finishes on its own
     * (returns false in isFinished) – caller must cancel.
     * 
     * @param velocity supplier producing target velocity for open-loop style control
     * @return non-terminating command until canceled
     */
    public Command setVelocity(Supplier<Double> velocity) {
        return new FunctionalCommand(
                () -> setVoltage(calculateVoltage(velocity.get())),
                () -> {
                },
                interrupted -> stop(),
                () -> false, this);
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
        builder.addDoubleProperty("setPoint", () -> goalState.position, this::setTarget);
        builder.addDoubleProperty("currentPosition", () -> motor.getEncoderPosition(), null);
        builder.addDoubleProperty("currentVelocity", () -> motor.getEncoderVelocity(), null);
        builder.addDoubleProperty("stowedPosition", () -> stowedPosition, null);
        builder.addDoubleProperty("clearedPosition", () -> clearedPosition, null);
        builder.addDoubleProperty("profiledPosition", () -> nextState.position, null);
        builder.addDoubleProperty("profiledVelocity", () -> nextState.velocity, null);
        builder.addDoubleProperty("measuredPosition", () -> lastMeasuredPos, null);
        builder.addDoubleProperty("measuredVelocity", () -> lastMeasuredVel, null);
        builder.addDoubleProperty("acceleration", () -> lastAcceleration, null);
        builder.addDoubleProperty("kP", () -> config.kP, v -> config.kP = v);
        builder.addDoubleProperty("kI", () -> config.kI, v -> config.kI = v);
        builder.addDoubleProperty("kD", () -> config.kD, v -> config.kD = v);
        builder.addDoubleProperty("kS", () -> config.kS, v -> config.kS = v);
        builder.addDoubleProperty("kV", () -> config.kV, v -> config.kV = v);
        builder.addDoubleProperty("kA", () -> config.kA, v -> config.kA = v);
        builder.addDoubleProperty("kG", () -> config.kG, v -> config.kG = v);
    }

    /**
     * Subclasses must create and configure the underlying motor (controller, inversion, current limiting, etc.).
     * 
     * @return initialized Motor instance
     */
    protected abstract Motor createMotor();

    /**
     * Provide a default command that continuously seeks current goal, stopping cleanly when not interrupted.
     * 
     * @return default command for this subsystem
     */
    protected Command createDefaultCommand() {
        Command defaultCommand = new FunctionalCommand(
                () -> {
                    if (!isInterupted) {
                        stop();
                    }
                },
                this::seekTarget,
                interrupted -> {
                    if (interrupted) {
                        interrupt();
                    } else {
                        stop();
                    }
                },
                this::atTarget,
                this);
        return defaultCommand;
    }

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

    // Reset internal PID accumulators (integral + last error). Can be invoked by subclasses when switching control modes.
    protected void resetPid() {
        pidIntegral       = 0.0;
        lastPositionError = 0.0;
    }

    // Compute braking distance needed to reduce current velocity to zero: v^2 / (2a)
    private double computeBrakingDistance(double velocity) {
        double v = Math.abs(velocity);
        return (v * v) / (2.0 * config.maximumAcceleration);
    }

    // Clamp a raw position to configured min/max limits
    private double clampToLimits(double position) {
        if (position < minimumSetPoint) {
            return minimumSetPoint;
        }
        if (position > maximumSetPoint) {
            return maximumSetPoint;
        }
        return position;
    }

    // Compute control voltage (PID + feedforward) or legacy feedforward only if no gains configured
    private double computeControlVoltage(State previousProfiledState, State measuredState) {
        boolean gainsConfigured = (config.kP != 0.0) || (config.kI != 0.0) || (config.kD != 0.0) || (config.kS != 0.0) ||
                (config.kV != 0.0) || (config.kA != 0.0) || (config.kG != 0.0);
        if (!gainsConfigured) {
            return calculateVoltageWithVelocities(previousProfiledState.velocity, nextState.velocity);
        }
        // Position error relative to profiled position
        double positionError = nextState.position - measuredState.position;
        // PID accumulation
        pidIntegral += positionError * kDt;
        // Anti-windup clamp (limit integral so kI * integral < 12V)
        if (config.kI != 0.0) {
            double maxIntegral = 12.0 / Math.abs(config.kI);
            if (pidIntegral > maxIntegral) {
                pidIntegral = maxIntegral;
            }
            if (pidIntegral < -maxIntegral) {
                pidIntegral = -maxIntegral;
            }
        }
        double derivative = (positionError - lastPositionError) / kDt;
        lastPositionError = positionError;
        double pid     = config.kP * positionError + config.kI * pidIntegral + config.kD * derivative;
        // Feedforward (sign-correct static term)
        double sign    = Math.signum(nextState.velocity == 0.0 ? positionError : nextState.velocity);
        double ff      = config.kS * sign + config.kV * nextState.velocity + config.kA * lastAcceleration + config.kG;
        double voltage = pid + ff;
        // Clamp output
        if (voltage > 12.0) {
            voltage = 12.0;
        }
        if (voltage < -12.0) {
            voltage = -12.0;
        }
        log.dashboardVerbose("pidTerm", pid);
        log.dashboardVerbose("ffTerm", ff);
        log.dashboardVerbose("controlVoltage", voltage);
        log.dashboardVerbose("positionError", positionError);
        return voltage;
    }
}
