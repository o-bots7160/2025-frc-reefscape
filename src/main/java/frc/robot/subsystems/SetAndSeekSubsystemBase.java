package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    protected double                  clearedPosition;

    protected double                  stowedPosition;

    protected TrapezoidProfile.State  goalState = new TrapezoidProfile.State();

    protected TrapezoidProfile        profile;

    protected double                  maximumSetPoint;

    protected double                  minimumSetPoint;

    protected Map<Integer, MotorData> motors    = new HashMap<>();

    protected double                  setPointTolerance;

    /**
     * The next state of the elevator. This needs to be captured at the class level as we will utilize it in the next cycle of the profile
     * calculation. Without it, or having it local, will cause eratic behavior on the trapezoidal profile
     */
    protected State                   nextState;

    protected SetAndSeekSubsystemBase(TConfig config) {
        super(config);

        minimumSetPoint   = config.minimumSetPoint;
        maximumSetPoint   = config.maximumSetPoint;
        setPointTolerance = config.setPointTolerance;
        clearedPosition   = config.clearedPosition;
        stowedPosition    = config.stowedPosition;

        nextState         = new State(0, 0);
        profile           = new TrapezoidProfile(new TrapezoidProfile.Constraints(config.maximumVelocity, config.maximumAcceleration));

        // Command defaultCommand = new FunctionalCommand(
        // // Nothing on init
        // () -> {
        // },
        // // When running, we'll seek to the last target
        // () -> seekTarget(),
        // // nothing on interrupt, since that means we're probably moving again
        // interrupted -> {
        // },
        // // this command ends if we're at target
        // () -> atTarget(), this);
        // setDefaultCommand(defaultCommand.unless(() -> atTarget()));
    }

    @Override
    public void periodic() {
        if (checkDisabled()) {
            return;
        }

        log.dashboardVerbose("goalPosition", goalState.position);
        log.dashboardVerbose("goalVelocity", goalState.velocity);

        for (Map.Entry<Integer, MotorData> entry : motors.entrySet()) {
            MotorData motorData = entry.getValue();
            log.dashboardVerbose(motorData.name + "Position", motorData.motor.getEncoderPosition());
        }
    }

    public void setTarget(double setPoint) {
        if (checkDisabled()) {
            return;
        }

        double newSetPoint = setPoint;

        if (setPoint < minimumSetPoint) {
            newSetPoint = minimumSetPoint;
        }

        if (setPoint > maximumSetPoint) {
            newSetPoint = maximumSetPoint;
        }

        nextState = new State(minimumSetPoint, nextState.velocity);
        goalState = new TrapezoidProfile.State(newSetPoint, 0.0);
    }

    public void setTargetVelocity(double velocity) {
        if (checkDisabled()) {
            return;
        }

        double baseFormula = Math.pow(nextState.velocity, 2.0) - Math.pow(velocity, 2.0) / 2.0 * config.maximumAcceleration;
        double setPoint    = nextState.velocity > velocity ? baseFormula : baseFormula * -1.0;

        setPoint = nextState.position + setPoint;
        setTarget(setPoint);
    }

    /**
     * Gets the current position of the system
     *
     * @return the position of the current state
     */
    public double getCurrentPosition() {
        return nextState.position;
    }

    /**
     * Seeks the target by calculating the voltage needed via the current {@link TrapezoidProfile.State} and next {@link TrapezoidProfile.State} of
     * the motor(s)
     */
    public void seekTarget() {
        if (checkDisabled()) {
            return;
        }
        State currentState = new State(getPrimaryMotor().getEncoderPosition(), nextState.velocity);

        nextState = profile.calculate(kDt, currentState, goalState);
        log.dashboardVerbose("currentState", currentState.velocity);
        log.dashboardVerbose("currentStatePosition", currentState.position);
        log.dashboardVerbose("nextState", nextState.velocity);
        log.dashboardVerbose("nextStatePosition", nextState.position);

        var calculatedVoltage = calculateVoltageWithVelocities(currentState.velocity, nextState.velocity);

        setVoltage(calculatedVoltage);
    }

    /*
     * Set the left elevator motor to the opposite of the right
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

    public void setVoltage(Voltage voltage) {
        if (checkDisabled()) {
            return;
        }

        setVoltage(voltage.baseUnitMagnitude());
    }

    /**
     * Returns true if the elevator is at a set point where it can be stowed
     *
     * @return True if the elevator is at a set point where it can be stowed
     */
    public boolean isStowed() {
        if (checkDisabled()) {
            return false;
        }

        double currentPosition = getPrimaryMotor().getEncoderPosition();
        return (currentPosition >= 0.0) && (currentPosition < stowedPosition);
    }

    /**
     * Checks if elevator is not too low to move manipulator
     *
     * @return true if elevator clear of stowing
     */
    public void setStow() {
        if (checkDisabled()) {
            return;
        }

        setTarget(stowedPosition);
    }

    /**
     * Checks if elevator is not too low to move manipulator
     *
     * @return true if elevator clear of stowing
     */
    public boolean isClear() {
        if (checkDisabled()) {
            return false;
        }

        return getPrimaryMotor().getEncoderPosition() > clearedPosition;
    }

    /**
     * Checks if elevator is not too low to move manipulator
     *
     * @return true if elevator clear of stowing
     */
    public void setClear() {
        if (checkDisabled()) {
            return;
        }

        setTarget(clearedPosition);
    }

    /**
     * Hold the elevator at the current set point
     *
     * @return void
     */
    public void hold() {
        if (checkDisabled()) {
            return;
        }

        var calculatedVoltage = calcuateVoltage(0.0);
        log.verbose("Calculated Voltage:" + calculatedVoltage);

        setVoltage(calculatedVoltage);
    }

    /**
     * Stop the elevator motors
     *
     * @return void
     */
    public void stop() {
        if (checkDisabled()) {
            return;
        }
        nextState.velocity = 0;

        Double stoppedVoltage = calcuateVoltage(0);
        setVoltage(stoppedVoltage);
    }

    public void slowStop() {

        // coast X distance as long as it's not beyond the original set point
        // otherwise, we're at the position and can stop normally if
        if (!atTarget()) {
            double  originalTarget = goalState.position;
            double  current        = nextState.position;
            boolean goingUp        = originalTarget > current;
            double  amountToMove   = Math.abs(originalTarget - current) *
                    0.25;
            double  newDestination = goingUp ? current + amountToMove : current - amountToMove;
            if ((goingUp && newDestination > originalTarget)
                    || (!goingUp && newDestination < originalTarget)) {
                // we went too far!
                newDestination = originalTarget;
            }
        } else {
            stop();
        }
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
     * Determines if the elevator is at the target set point
     *
     * @return True if the elevator is at the target set point
     */
    public boolean atTarget() {
        if (checkDisabled()) {
            return false;
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
     * Determines if the subsystem must reverse to reach the target
     *
     * @return True if the target requires reversing direction
     */
    public boolean reverseDirection(double nextTarget) {
        if (checkDisabled()) {
            return false;
        }

        var     motor       = getPrimaryMotor();
        boolean returnValue = false;
        double  direction   = nextTarget - motor.getEncoderPosition();

        if (direction > 0.0 && motor.getEncoderVelocity() < -1.0) {
            returnValue = true;
        } else if (direction < 0.0 && motor.getEncoderVelocity() > 1.0) {
            returnValue = true;
        }
        return returnValue;
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
                interrupted -> stop(),
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
    protected abstract double calcuateVoltage(double velocity);

    /**
     * Logs elevator motor activity for SysId
     *
     * @param log used to collect data
     * @return void
     */
    private void logActivity(SysIdRoutineLog routineLog) {
        var motor = getPrimaryMotor();
        routineLog.motor("shoulder").voltage(motor.getVoltage())
                .angularPosition(Units.Degrees.of(motor.getEncoderPosition()))
                .angularVelocity(Units.DegreesPerSecond.of(motor.getEncoderVelocity()));
    }

}
