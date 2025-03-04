package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.commands.ElevatorCommand;
import frc.robot.config.SubsystemsConfig;
import frc.robot.devices.LinearMotor;

/**
 *
 */
@Logged
public class ElevatorSubsystem extends ObotSubsystemBase {

    public BooleanSupplier         clearToStow = () -> {
                                                   return true;
                                               };

    // kS, kG, kV, kA TODO: Run with shoulder attached
    ElevatorFeedforward            feedforward = new ElevatorFeedforward(1.0, 1.0, 1.0, 1.0);

    private LinearMotor            rightElevatorMotor;

    private LinearMotor            leftElevatorMotor;

    private final double           kDt         = 0.02;

    // meters TODO: Measure value on robot
    private final double           clearHeight = 6.0;

    private final double           minHeight   = 3.0;

    // meters TODO: Measure value on robot
    private final double           maxHeight   = 150.0;

    // TODO: Max speed/accel?
    private final TrapezoidProfile profile     = new TrapezoidProfile(new TrapezoidProfile.Constraints(5.0, 0.75));

    private TrapezoidProfile.State goal        = new TrapezoidProfile.State();

    private TrapezoidProfile.State setpoint    = new TrapezoidProfile.State();

    /**
    *
    */
    public ElevatorSubsystem(SubsystemsConfig subsystemsConfig) {
        super(subsystemsConfig);
        rightElevatorMotor = new LinearMotor(52, 6, 150);
        leftElevatorMotor  = new LinearMotor(53, 6, 150);
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    @Override
    public void periodic() {
        // atTarget();

        //log.dashboardVerbose("setpointPosition", setpoint.position);
        //log.dashboardVerbose("goalPosition", goal.position);
        //log.dashboardVerbose("actualPosition", rightElevatorMotor.getEncoderPosition());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void setTarget(double new_height) {
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
        setpoint = profile.calculate(kDt, setpoint, goal);

        var calculatedVoltage = feedforward.calculateWithVelocities(rightElevatorMotor.getEncoderVelocity(),
                setpoint.velocity);
        log.dashboardVerbose("calculatedVoltage", calculatedVoltage);

        setVoltages(calculatedVoltage);
    }

    /**
     * Sets a fixed command
     *
     * @return void
     */
    public void setConstant(double volts) {
        setVoltages(volts);
    }

    /**
     * Hold the elevator at the current height
     *
     * @return void
     */
    public void hold() {
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
        setVoltages(0.0);
    }

    /*
     * Set the left elevator motor to the opposite of the right
     * @return void
     */
    public void setVoltages(double voltage) {
        rightElevatorMotor.setVoltage(voltage);
        leftElevatorMotor.setVoltage(-voltage);
    }

    /**
     * Determines if the elevator is at the target height
     *
     * @return True if the elevator is at the target height
     */
    public boolean atTarget() {
        var degreesDifference   = setpoint.position - goal.position;
        var marginOfError       = Math.abs(degreesDifference);

        var withinMarginOfError = marginOfError < 1.0;
        log.dashboardVerbose("marginOfError", marginOfError);

        return withinMarginOfError;
    }

    /**
     * Returns true if the elevator is at a height where it can be stowed
     *
     * @return True if the elevator is at a height where it can be stowed
     */
    public boolean isStowed() {
        double centimeters = setpoint.position;
        return (centimeters >= 0.0) && (centimeters < 2.0);
    }

    /**
     * Checks if elevator is not too low to move manipulator
     *
     * @return true if elevator clear of stowing
     */
    public boolean isClear() {
        return rightElevatorMotor.getEncoderPosition() > clearHeight;
    }

    public Command stowCommand() {
        return new ElevatorCommand(this, 0);
    }

    public Command goToCommand(double position) {
        return new ElevatorCommand(this, position);
    }

    public Command goToCommand(Supplier<Double> position) {
        return new ElevatorCommand(this, position);
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
     * Creates a command that can be mapped to a button or other trigger. Delays can
     * be set to customize the length of each part of the SysId Routine
     *
     * @param config         - The Sys Id routine runner
     * @param subsystem      - seconds between each portion to allow motors to spin
     *                       down, etc...
     * @param quasiTimeout   - seconds to run the Quasistatic routines, so robot
     *                       doesn't get too far
     * @param dynamicTimeout - seconds to run the Dynamic routines, 2-3 secs should
     *                       be enough
     * @param maxVolts       - The maximum voltage that should be applied to the
     *                       drive motors.
     * @return A command that can be mapped to a button or other trigger
     */
    private SysIdRoutine setSysIdRoutine(Config config) {
        return new SysIdRoutine(config, new SysIdRoutine.Mechanism(
                (volts) -> this.setVoltage(volts.baseUnitMagnitude()), (log) -> this.logActivity(log), this));
    }

}
