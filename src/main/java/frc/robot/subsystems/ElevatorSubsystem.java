package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.commands.TestLoggerCommand;
import frc.robot.commands.elevator.ElevatorCommand;
import frc.robot.config.ElevatorSubsystemConfig;
import frc.robot.config.SubsystemsConfig;
import frc.robot.devices.LinearMotor;

/**
 *
 */
@Logged
public class ElevatorSubsystem extends ObotSubsystemBase<ElevatorSubsystemConfig> {

    public BooleanSupplier         clearToStow = () -> {
                                                   return true;
                                               };

    // kS, kG, kV, kA TODO: Run with shoulder attached
    ElevatorFeedforward            feedforward = new ElevatorFeedforward(0.5, 0.5, 0.5, 0.5);

    private LinearMotor            rightElevatorMotor;

    private LinearMotor            leftElevatorMotor;

    private final double           kDt         = 0.02;

    // meters TODO: Measure value on robot
    private final double           clearHeight = 6.0;

    private final double           minHeight   = 0.0;

    // meters TODO: Measure value on robot
    private final double           maxHeight   = 50.0;

    // TODO: Max speed/accel?
    private final TrapezoidProfile profile     = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.0, 0.25));

    private TrapezoidProfile.State goal        = new TrapezoidProfile.State();

    private TrapezoidProfile.State setpoint    = new TrapezoidProfile.State();

    /**
    *
    */
    public ElevatorSubsystem(SubsystemsConfig subsystemsConfig) {
        super(subsystemsConfig.elevatorSubsystem);
        if (checkDisabled()) {
            return;
        }

        rightElevatorMotor = new LinearMotor(config.rightMotorCanId, 6, 150);
        leftElevatorMotor  = new LinearMotor(config.leftMotorCanId, 6, 150);
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    @Override
    public void periodic() {
        if (checkDisabled()) {
            return;
        }

        atTarget();

        log.dashboardVerbose("setpointPosition", setpoint.position);
        log.dashboardVerbose("goalPosition", goal.position);
        log.dashboardVerbose("rightMotorActualPosition", rightElevatorMotor.getEncoderPosition());
        log.dashboardVerbose("leftMotorActualPosition", leftElevatorMotor.getEncoderPosition());
    }

    public void setTarget(double new_height) {
        if (checkDisabled()) {
            return;
        }

        double height = new_height;

        if (new_height < minHeight) {
            height = minHeight;
        } else if (new_height > maxHeight) {
            height = maxHeight;
        }
        goal = new TrapezoidProfile.State(height, 0.0);
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
        setpoint = new State(rightElevatorMotor.getEncoderPosition(), rightElevatorMotor.getEncoderVelocity());

        setpoint = profile.calculate(kDt, setpoint, goal);

        var calculatedVoltage = feedforward.calculateWithVelocities(rightElevatorMotor.getEncoderVelocity(), setpoint.velocity);
        if ((rightElevatorMotor.getEncoderPosition() < minHeight) &&
                (calculatedVoltage < 0.0)) {
            calculatedVoltage = 0.0;
        } else if ((rightElevatorMotor.getEncoderPosition() > maxHeight) &&
                (calculatedVoltage > 0.0)) {
            calculatedVoltage = 0.0;
        }
        log.dashboardVerbose("calculatedVoltage", calculatedVoltage);

        setVoltages(calculatedVoltage);
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

        setVoltages(volts);
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

        setVoltages(calculatedVoltage);
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

        setVoltages(0.0);
    }

    /*
     * Set the left elevator motor to the opposite of the right
     * @return void
     */
    public void setVoltages(double voltage) {
        if (checkDisabled()) {
            return;
        }

        log.verbose("setting voltages of motors to " + voltage);
        rightElevatorMotor.setVoltage(voltage);
        leftElevatorMotor.setVoltage(voltage);
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

        var lengthDifference    = setpoint.position - goal.position;
        var marginOfError       = Math.abs(lengthDifference);

        var withinMarginOfError = marginOfError < 5.0;
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

        double centimeters = setpoint.position;
        return (centimeters >= 0.0) && (centimeters < 2.0);
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

    public Command stowCommand() {
        if (checkDisabled()) {
            return new TestLoggerCommand("stowCommand method not called");
        }

        return new ElevatorCommand(this, 0);
    }

    public Command goToCommand(double position) {
        if (checkDisabled()) {
            return new TestLoggerCommand("goToCommand method not called");
        }

        return new ElevatorCommand(this, position);
    }

    public Command goToCommand(Supplier<Double> position) {
        if (checkDisabled()) {
            return new TestLoggerCommand("goToCommand method not called");
        }

        return new ElevatorCommand(this, position);
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

        SysIdRoutine routine = setSysIdRoutine(new Config());

        return routine
                .quasistatic(SysIdRoutine.Direction.kForward).withTimeout(quasiTimeout)
                .until(() -> {
                    log.debug("quasi - right motor position: " + rightElevatorMotor.getEncoderPosition());
                    return rightElevatorMotor.getEncoderPosition() > maxHeight;
                }).andThen(Commands.waitSeconds(delay))
                .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(quasiTimeout))
                .until(() -> rightElevatorMotor.getEncoderPosition() < minHeight).andThen(Commands.waitSeconds(delay))

                .andThen(routine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(dynamicTimeout))
                .until(() -> rightElevatorMotor.getEncoderPosition() > maxHeight).andThen(Commands.waitSeconds(delay))
                .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(dynamicTimeout))
                .until(() -> rightElevatorMotor.getEncoderPosition() < minHeight);
    }

    /**
     * Sets motor voltages
     *
     * @param new_voltage that the elevator needs to go to
     * @return void
     */
    private void setVoltage(double new_voltage) {
        setVoltages(new_voltage);
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

    /**
     * Creates a command that can be mapped to a button or other trigger. Delays can be set to customize the length of each part of the SysId Routine
     *
     * @param config         - The Sys Id routine runner
     * @param subsystem      - seconds between each portion to allow motors to spin down, etc...
     * @param quasiTimeout   - seconds to run the Quasistatic routines, so robot doesn't get too far
     * @param dynamicTimeout - seconds to run the Dynamic routines, 2-3 secs should be enough
     * @param maxVolts       - The maximum voltage that should be applied to the drive motors.
     * @return A command that can be mapped to a button or other trigger
     */
    private SysIdRoutine setSysIdRoutine(Config config) {
        return new SysIdRoutine(config,
                new SysIdRoutine.Mechanism((volts) -> this.setVoltage(volts.baseUnitMagnitude()), (log) -> this.logActivity(log), this));
    }

}
