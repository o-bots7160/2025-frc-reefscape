package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.config.SetAndSeekSubsystemConfigBase;
import frc.robot.devices.MotorBase;

public abstract class SetAndSeekSubsystemBase<TConfig extends SetAndSeekSubsystemConfigBase> extends ObotSubsystemBase<TConfig> {

    protected static class MotorData {
        MotorBase motor;

        String    name;

        double    conversionFactor;

        MotorData(MotorBase motor, String name, double conversionFactor) {
            this.motor            = motor;
            this.name             = name;
            this.conversionFactor = conversionFactor;
        }

        MotorData(MotorBase motor, String name) {
            this(motor, name, 1.0);
        }
    }

    protected int                     verboseDelay = 0;

    private boolean                   requestStop  = false;

    protected double                  clearedPosition;

    protected double                  stowedPosition;

    protected TrapezoidProfile.State  goalState    = new TrapezoidProfile.State();

    protected TrapezoidProfile        profile;

    protected double                  maximumSetPoint;

    protected double                  minimumSetPoint;

    protected Map<Integer, MotorData> motors       = new HashMap<>();

    protected double                  setPointTolerance;

    protected double                  stoppingTolerance;

    protected boolean                 isInterupted = false;

    /**
     * The next state of the subsystem. This needs to be captured at the class level as we will utilize it in the next cycle of the profile
     * calculation. Without it, or having it local, will cause eratic behavior on the trapezoidal profile
     */
    protected State                   nextState;

    protected SetAndSeekSubsystemBase(TConfig config) {
        super(config);

        minimumSetPoint   = config.minimumSetPoint;
        maximumSetPoint   = config.maximumSetPoint;
        setPointTolerance = config.setPointTolerance;
        clearedPosition   = config.clearedPosition;
        stoppingTolerance = config.stoppingTolerance;
        stowedPosition    = config.stowedPosition;

        nextState         = new State(0, 0);
        profile           = new TrapezoidProfile(new TrapezoidProfile.Constraints(config.maximumVelocity, config.maximumAcceleration));

        // if (config.enableDefaultCommand) {
        // Command defaultCommand = createDefaultCommand();
        // setDefaultCommand(defaultCommand);
        // }
        (new Trigger(() -> requestStop)).onTrue(seekZeroVelocity());
    }

    @Override
    public void periodic() {
        if (checkDisabled()) {
            return;
        }

        if ( ( ++verboseDelay % 200 ) == 0 )
        {
            log.dashboardVerbose("goalPosition", goalState.position);
            log.dashboardVerbose("goalVelocity", goalState.velocity);
            log.dashboardVerbose("requestStop", requestStop);

            for (Map.Entry<Integer, MotorData> entry : motors.entrySet()) {
                MotorData motorData = entry.getValue();
                log.dashboardVerbose(motorData.name + "Position", motorData.motor.getEncoderPosition());
            }
        }
    }

    public void setTarget(double setPoint) {
        if (checkDisabled()) {
            return;
        }

        isInterupted = false;

        double newSetPoint = setPoint;

        if (setPoint < minimumSetPoint) {
            newSetPoint = minimumSetPoint;
        }

        if (setPoint > maximumSetPoint) {
            newSetPoint = maximumSetPoint;
        }

        nextState   = new State(minimumSetPoint, nextState.velocity);
        goalState   = new TrapezoidProfile.State(newSetPoint, 0.0);

        requestStop = false;
    }

    public void setTargetVelocity(double velocity) {
        if (checkDisabled()) {
            return;
        }

        double baseFormula = (Math.pow(nextState.velocity, 2.0) - Math.pow(velocity, 2.0)) / 2.0 * config.maximumAcceleration;
        double setPoint    = nextState.velocity > velocity ? baseFormula : baseFormula * -1.0;

        setPoint = nextState.position + setPoint;
        setTarget(setPoint);
    }

    public void interrupt() {
        var currentPosition = getCurrentPosition();
        var currentVelocity = getCurrentVelocity();

        if (currentVelocity < 0) {
            setTarget(currentPosition - stoppingTolerance);
        } else {
            setTarget(currentPosition + stoppingTolerance);
        }

        isInterupted = true;
    }

    public boolean isInterupted() {
        return isInterupted;
    }

    /**
     * Gets the current position of the subsystem
     *
     * @return the position of the current state
     */
    public double getCurrentPosition() {
        return nextState.position;
    }

    /**
     * Gets the current velocity of the subsystem
     *
     * @return the velocity of the current state
     */
    public double getCurrentVelocity() {
        return nextState.velocity;
    }

    /**
     * Seeks the target by calculating the voltage needed via the current {@link TrapezoidProfile.State} and next {@link TrapezoidProfile.State} of
     * the motor(s)
     * 
     * @return the calculated voltage
     */
    public double seekTarget() {
        if (checkDisabled()) {
            return 0.0;
        }
        State currentState = new State(getPrimaryMotor().getEncoderPosition(), nextState.velocity);

        nextState = profile.calculate(kDt, currentState, goalState);
        log.dashboardVerbose("currentState", currentState.velocity);
        log.dashboardVerbose("currentStatePosition", currentState.position);
        log.dashboardVerbose("nextState", nextState.velocity);
        log.dashboardVerbose("nextStatePosition", nextState.position);

        var calculatedVoltage = calculateVoltageWithVelocities(currentState.velocity, nextState.velocity);

        setVoltage(calculatedVoltage);

        return calculatedVoltage;
    }

    /**
     * Set the subsystem motor voltages
     * 
     * @return void
     */
    public void setVoltage(double voltage) {
        if (checkDisabled()) {
            return;
        }

        log.dashboardVerbose("setVoltage", voltage);
        for (MotorData motorData : motors.values()) {
            motorData.motor.setVoltage(voltage * motorData.conversionFactor);
        }
    }

    /**
     * Returns true if the subsystem is at a set point where it can be stowed
     *
     * @return True if the subsystem is at a set point where it can be stowed
     */
    public boolean isStowed() {
        if (checkDisabled()) {
            return false;
        }

        double currentPosition = getPrimaryMotor().getEncoderPosition();
        return (currentPosition >= 0.0) && (currentPosition < stowedPosition);
    }

    /**
     * Sets the subsystem to a position that is stowed
     *
     * @return true if subsystem clear of stowing
     */
    public void setStow() {
        if (checkDisabled()) {
            return;
        }

        setTarget(stowedPosition);
    }

    /**
     * Checks if subsystem is clear to operate
     *
     * @return true if subsystem clear of stowing
     */
    public boolean isClear() {
        if (checkDisabled()) {
            return true;
        }

        return getPrimaryMotor().getEncoderPosition() > clearedPosition;
    }

    /**
     * Sets the subsystem to a position that is clear to operate
     *
     * @return true if subsystem clear of stowing
     */
    public void setClear() {
        if (checkDisabled()) {
            return;
        }

        setTarget(clearedPosition);
    }

    /**
     * Hold the subsystem at the current set point
     *
     * @return void
     */
    public void hold() {
        if (checkDisabled()) {
            return;
        }

        var calculatedVoltage = calculateVoltage(0.0);
        log.verbose("Calculated Voltage:" + calculatedVoltage);

        setVoltage(calculatedVoltage);
    }

    /**
     * Stop the subsystem motors
     *
     * @return void
     */
    public void stop() {
        if (checkDisabled()) {
            return;
        }
        nextState.velocity = 0;

        Double stoppedVoltage = calculateVoltage(0);
        setVoltage(stoppedVoltage);
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

        setVoltage(volts);
    }

    /**
     * Sets a fixed command
     *
     * @return void
     */
    public void requestStop() {
        requestStop = true;
    }

    /**
     * Determines if the subsystem is at the target set point
     *
     * @return True if the subsystem is at the target set point
     */
    public boolean atTarget() {
        if (checkDisabled()) {
            return true;
        }

        var   motor               = getPrimaryMotor();
        State currentState        = new State(motor.getEncoderPosition(), motor.getEncoderVelocity());

        var   lengthDifference    = currentState.position - goalState.position;
        var   marginOfError       = Math.abs(lengthDifference);

        var   withinMarginOfError = marginOfError < setPointTolerance;
        log.dashboardVerbose("setPointTolerance", setPointTolerance);
        log.dashboardVerbose("marginOfError", marginOfError);

        return withinMarginOfError;
    }

    /**
     * Returns a command that seeks 0 velocity and stops
     *
     * @return Command
     */
    public Command seekTarget(Supplier<Double> position) {
        return new FunctionalCommand(
                () -> setTarget(position.get()),
                () -> seekTarget(),
                interrupted -> {
                    if (interrupted) {
                        requestStop = true;
                    } else {
                        stop();
                    }
                },
                () -> atTarget(), this);
    }

    /**
     * Returns a command that seeks 0 velocity and stops
     *
     * @return Command
     */
    public Command seekZeroVelocity() {
        return new FunctionalCommand(
                () -> setTargetVelocity(0.0),
                () -> seekTarget(),
                interrupted -> {
                    if (interrupted) {
                        requestStop = true;
                    } else {
                        stop();
                    }
                },
                () -> atTarget(), this);
    }

    /**
     * Creates a command that can be mapped to a button or other trigger. Delays can be set to customize the length of each part of the SysId Routine
     *
     * @param delay          - seconds between each portion to allow motors to spin down, etc...
     * @param quasiTimeout   - seconds to run the Quasistatic routines, so robot doesn't get too far
     * @param dynamicTimeout - seconds to run the Dynamic routines, 2-3 secs should be enough
     * @return A command that can be mapped to a button or other trigger
     */
    public Command generateSysIdCommand(double delay, double quasiTimeout, double dynamicTimeout) {
        if (checkDisabled()) {
            return new InstantCommand(() -> log.verbose("generateSysIdCommand method not called"));
        }

        Config                 sysIdRoutineConfig = new Config();
        SysIdRoutine.Mechanism sysIdMechanism     = new SysIdRoutine.Mechanism((v) -> setVoltage(v.baseUnitMagnitude()), this::logActivity,
                this);
        SysIdRoutine           routine            = new SysIdRoutine(sysIdRoutineConfig, sysIdMechanism);

        var                    motor              = getPrimaryMotor();

        return routine
                // Quasi Forward
                .quasistatic(SysIdRoutine.Direction.kForward)
                .until(() -> motor.getEncoderPosition() > maximumSetPoint)
                .withTimeout(quasiTimeout)
                .andThen(Commands.waitSeconds(delay))
                // Quasi Reverse
                .andThen(
                        routine.quasistatic(SysIdRoutine.Direction.kReverse)
                                .until(() -> motor.getEncoderPosition() < minimumSetPoint)
                                .withTimeout(quasiTimeout))

                .andThen(Commands.waitSeconds(delay))
                // Dynamic Forward
                .andThen(
                        routine.dynamic(SysIdRoutine.Direction.kForward)
                                .until(() -> motor.getEncoderPosition() > maximumSetPoint)
                                .withTimeout(dynamicTimeout))

                .andThen(Commands.waitSeconds(delay))
                // Dynamic Reverse
                .andThen(
                        routine.dynamic(SysIdRoutine.Direction.kReverse)
                                .until(() -> motor.getEncoderPosition() < minimumSetPoint)
                                .withTimeout(dynamicTimeout));
    }

    protected MotorBase getPrimaryMotor() {
        return motors.get(0).motor;
    }

    protected Command createDefaultCommand() {
        Command defaultCommand = new FunctionalCommand(
                () -> {
                    if (!isInterupted) {
                        stop();
                    }
                },
                // We're going to seek the target if we're interrupted, otherwise nothing
                this::seekTarget,
                interrupted -> {
                    stop();
                },
                this::atTarget,
                this);

        return defaultCommand;
    }

    /**
     * Calculate the needed voltage using the current and next velocity via a feed forward mechanism
     *
     * @param currentVelocity the velocity from the current {@link TrapezoidProfile.State} of the motor
     * @param nextVelocity    the velocity from the next {@link TrapezoidProfile.State} of the motor
     * @return
     */
    protected abstract double calculateVoltageWithVelocities(double currentVelocity, double nextVelocity);

    /**
     * Calculate the needed voltage using the given velocity via a feed forward mechanism
     *
     * @param velocity the expected velocity of the motor
     * @return
     */
    protected abstract double calculateVoltage(double velocity);

    /**
     * Logs subsystem motor activity for SysId
     *
     * @param log used to collect data
     * @return void
     */
    protected void logActivity(SysIdRoutineLog routineLog) {
        var motor = getPrimaryMotor();
        routineLog.motor("shoulder").voltage(motor.getVoltage())
                .angularPosition(Units.Degrees.of(motor.getEncoderPosition()))
                .angularVelocity(Units.DegreesPerSecond.of(motor.getEncoderVelocity()));
    }

}
