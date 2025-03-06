package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.commands.TestLoggerCommand;
import frc.robot.commands.elevator.MoveElevatorCommand;
import frc.robot.config.ElevatorSubsystemConfig;
import frc.robot.config.SubsystemsConfig;
import frc.robot.devices.LinearMotor;

/**
 *
 */
@Logged
public class ElevatorSubsystem extends ObotSubsystemBase<ElevatorSubsystemConfig> {

    private double                 clearHeight;

    private double                 maxHeight;

    private double                 minHeight;

    private double                 stowHeight;

    private ElevatorFeedforward    feedforward = new ElevatorFeedforward(0.5, 0.0, 1.0, 0.5);

    private final double           kDt         = 0.02;

    private final TrapezoidProfile profile     = new TrapezoidProfile(new TrapezoidProfile.Constraints(5.0, 0.25));

    private LinearMotor            leftElevatorMotor;

    private LinearMotor            rightElevatorMotor;

    private TrapezoidProfile.State goalState   = new TrapezoidProfile.State();

    /**
    *
    */
    public ElevatorSubsystem(SubsystemsConfig subsystemsConfig) {
        super(subsystemsConfig.elevatorSubsystem);
        if (checkDisabled()) {
            return;
        }

        clearHeight        = config.clearHeight;
        stowHeight         = config.stowHeight;
        minHeight          = config.minimumHeight;
        maxHeight          = config.maximumHeight;

        rightElevatorMotor = new LinearMotor(config.rightMotorCanId, minHeight, maxHeight);
        leftElevatorMotor  = new LinearMotor(config.leftMotorCanId, minHeight, maxHeight);
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    @Override
    public void periodic() {
        if (checkDisabled()) {
            return;
        }

        atTarget();

        log.dashboardVerbose("goalPosition", goalState.position);
        log.dashboardVerbose("goalVelocity", goalState.velocity);
        log.dashboardVerbose("rightMotorActualPosition", rightElevatorMotor.getEncoderPosition());
        log.dashboardVerbose("leftMotorActualPosition", leftElevatorMotor.getEncoderPosition());
    }

    public void setTarget(double height) {
        if (checkDisabled()) {
            return;
        }

        double newHeight = height;

        if (height < minHeight) {
            newHeight = minHeight;
        }

        if (height > maxHeight) {
            newHeight = maxHeight;
        }
        goalState = new TrapezoidProfile.State(newHeight, 0.0);
    }

    /**
     * Seeks the target height for the elevator
     *
     * @return void
     */
    public void seekTarget() {
        if (checkDisabled()) {
            return;
        }
        State currentState      = new State(rightElevatorMotor.getEncoderPosition(), rightElevatorMotor.getEncoderVelocity());

        State nextState         = profile.calculate(kDt, currentState, goalState);

        var   calculatedVoltage = feedforward.calculateWithVelocities(currentState.velocity, nextState.velocity);

        // If we are under the minimum height and set to go down, we want to stop ASAP
        if ((currentState.position < minHeight) &&
                (calculatedVoltage < 0.0)) {
            log.warning("Minimum height below with a negative voltage; setting to 0.");
            calculatedVoltage = 0.0;
        }

        // If we are over the maximum height and set to go up, we want to stop ASAP
        if ((currentState.position > maxHeight) &&
                (calculatedVoltage > 0.0)) {
            log.warning("Maximum height above with a positive voltage; setting to 0.");
            calculatedVoltage = 0.0;
        }

        setVoltage(calculatedVoltage);
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
     * Hold the elevator at the current height
     *
     * @return void
     */
    public void hold() {
        if (checkDisabled()) {
            return;
        }

        var calculatedVoltage = feedforward.calculate(0.0);
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

        setVoltage(0.0);
    }

    /*
     * Set the left elevator motor to the opposite of the right
     * @return void
     */
    public void setVoltage(double voltage) {
        if (checkDisabled()) {
            return;
        }

        double actualVoltage = -1.0 * voltage;

        log.dashboardVerbose("setVoltage", voltage);
        log.dashboardVerbose("actualVoltage", actualVoltage);
        rightElevatorMotor.setVoltage(actualVoltage);
        leftElevatorMotor.setVoltage(actualVoltage);
    }

    public void setVoltage(Voltage voltage) {
        if (checkDisabled()) {
            return;
        }

        setVoltage(voltage.baseUnitMagnitude());
    }

    /**
     * Determines if the elevator is at the target height
     *
     * @return True if the elevator is at the target height
     */
    public boolean atTarget() {
        if (checkDisabled()) {
            return false;
        }

        State currentState        = new State(rightElevatorMotor.getEncoderPosition(), rightElevatorMotor.getEncoderVelocity());

        var   lengthDifference    = currentState.position - goalState.position;
        var   marginOfError       = Math.abs(lengthDifference);

        var   withinMarginOfError = marginOfError < 5.0;
        log.dashboardVerbose("marginOfError", marginOfError);

        return withinMarginOfError;
    }

    /**
     * Returns true if the elevator is at a height where it can be stowed
     *
     * @return True if the elevator is at a height where it can be stowed
     */
    public boolean isStowed() {
        if (checkDisabled()) {
            return false;
        }

        double centimeters = rightElevatorMotor.getEncoderPosition();
        return (centimeters >= 0.0) && (centimeters < stowHeight);
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

        return rightElevatorMotor.getEncoderPosition() > clearHeight;
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

        setTarget(clearHeight);
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

        setTarget(stowHeight);
    }

    public Command goToCommand(double position) {
        if (checkDisabled()) {
            return new TestLoggerCommand("goToCommand method not called");
        }

        return new MoveElevatorCommand(this, position);
    }

    public Command goToCommand(Supplier<Double> position) {
        if (checkDisabled()) {
            return new TestLoggerCommand("goToCommand method not called");
        }

        return new MoveElevatorCommand(this, position);
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
            return new TestLoggerCommand("generateSysIdCommand method not called");
        }

        Config                 sysIdRoutineConfig = new Config();
        SysIdRoutine.Mechanism sysIdMechanism     = new SysIdRoutine.Mechanism((v) -> setVoltage(v.baseUnitMagnitude() * -1.0), this::logActivity,
                this);
        SysIdRoutine           routine            = new SysIdRoutine(sysIdRoutineConfig, sysIdMechanism);

        return routine
                // Quasi Forward
                .quasistatic(SysIdRoutine.Direction.kForward)
                .until(() -> rightElevatorMotor.getEncoderPosition() > maxHeight)
                .withTimeout(quasiTimeout)
                .andThen(Commands.waitSeconds(delay))
                // Quasi Reverse
                .andThen(
                        routine.quasistatic(SysIdRoutine.Direction.kReverse)
                                .until(() -> rightElevatorMotor.getEncoderPosition() < minHeight)
                                .withTimeout(quasiTimeout))

                .andThen(Commands.waitSeconds(delay))
                // Dynamic Forwa
                .andThen(
                        routine.dynamic(SysIdRoutine.Direction.kForward)
                                .until(() -> rightElevatorMotor.getEncoderPosition() > maxHeight)
                                .withTimeout(dynamicTimeout))

                .andThen(Commands.waitSeconds(delay))
                // Dynamic Reverse
                .andThen(
                        routine.dynamic(SysIdRoutine.Direction.kReverse)
                                .until(() -> rightElevatorMotor.getEncoderPosition() < minHeight)
                                .withTimeout(dynamicTimeout));
    }

    /**
     * Logs elevator motor activity for SysId
     *
     * @param log used to collect data
     * @return void
     */
    private void logActivity(SysIdRoutineLog routineLog) {
        routineLog.motor("shoulder").voltage(rightElevatorMotor.getVoltage())
                .angularPosition(Units.Degrees.of(rightElevatorMotor.getEncoderPosition()))
                .angularVelocity(Units.DegreesPerSecond.of(rightElevatorMotor.getEncoderVelocity()));
    }

}
