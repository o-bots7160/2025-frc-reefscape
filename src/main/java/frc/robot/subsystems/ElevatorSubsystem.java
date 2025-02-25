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
import frc.robot.devices.PositionalMotor;

/**
 *
 */
@Logged
public class ElevatorSubsystem extends ObotSubsystemBase {

    // kS, kG, kV, kA TODO: do we need these for loaded intakes?
    ElevatorFeedforward            feedforward    = new ElevatorFeedforward(1.0, 1.0, 1.0, 1.0);

    private PositionalMotor        rightElevatorMotor;

    private PositionalMotor        leftElevatorMotor;

    private final double           kDt            = 0.02;

    // meters TODO: Measure value on robot
    private final double           clearHeight    = 6.0;

    private final double           minHeight      = 3.0;

    // meters TODO: Measure value on robot
    private final double           maxHeight      = 150.0;

    // TODO: Max speed/accel?
    private final TrapezoidProfile profile        = new TrapezoidProfile(new TrapezoidProfile.Constraints(5.0, 0.75));

    private TrapezoidProfile.State goal           = new TrapezoidProfile.State();

    private TrapezoidProfile.State setpoint       = new TrapezoidProfile.State();

    public BooleanSupplier         clearToStow    = () -> {
                                                      return true;
                                                  };

    /**
    *
    */
    public ElevatorSubsystem() 
    {
        rightElevatorMotor = new PositionalMotor(52, 6, 150);
        leftElevatorMotor  = new PositionalMotor(53, 6, 150);
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    @Override
    public void periodic() {
        /*
        TrapezoidProfile.State temp_goal = goal;

        // check if the rest of the manipulator is clear to stow. if
        // not, only allow the elevator to go down to the clearHeight
        if ((goal.position < clearHeight) && !clearToStow.getAsBoolean()) {
            temp_goal = new TrapezoidProfile.State(clearHeight, 0.0);
        }
        setpoint = new TrapezoidProfile.State(rightElevatorMotor.getEncoderPosition(), rightElevatorMotor.getEncoderVelocity());
        setpoint = profile.calculate(kDt, setpoint, temp_goal);

        setVoltage(feedforward.calculate(setpoint.velocity));

        if (!home.get()) // Home elevator if down limit switch is made
        {
            encoder.setPosition(0.0);
        }
        if (verbosity) {
            SmartDashboard.putNumber("elevator/target", goal.position);
            SmartDashboard.putNumber("elevator/position", rightElevatorMotor.getEncoderPosition());
            SmartDashboard.putNumber("elevator/velocity", rightElevatorMotor.getEncoderVelocity());
            SmartDashboard.putBoolean("elevator/reset", home.get());
        }
        SmartDashboard.putBoolean("elevator/at_target", atTarget());
        */
        atTarget();

        log.dashboardVerbose("setpointPosition", setpoint.position);
        log.dashboardVerbose("goalPosition", goal.position);
        log.dashboardVerbose("actualPosition", rightElevatorMotor.getEncoderPosition());
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
     * Sets motor voltages
     *
     * @param new_voltage that the elevator needs to go to
     * @return void
     */
    private void setVoltage(double new_voltage) {
        rightElevatorMotor.setVoltage(new_voltage);
    }

    /**
     * Seeks the target angle for the shoulder
     *
     * @return void
     */
    public void seekTarget() {
        setpoint = profile.calculate(kDt, setpoint, goal);

        var calculatedVoltage = feedforward.calculateWithVelocities(rightElevatorMotor.getEncoderVelocity(), setpoint.velocity);
        log.dashboardVerbose("calculatedVoltage", calculatedVoltage);

        rightElevatorMotor.setVoltage(calculatedVoltage);
        leftElevatorMotor.setVoltage(-calculatedVoltage);
    }
    /**
     * Sets a fixed command
     *
     * @return void
     */
    public void setConstant( double volts) {
        rightElevatorMotor.setVoltage(volts);
        leftElevatorMotor.setVoltage(-volts);
    }
    /**
     * Hold the shoulder at the current angle
     *
     * @return void
     */
    public void hold() {
        var calculatedVoltage = feedforward.calculate(0.0);
        log.verbose("Calculated Voltage:" + calculatedVoltage);

        rightElevatorMotor.setVoltage(calculatedVoltage);
        leftElevatorMotor.setVoltage(-calculatedVoltage);
    }
    /**
     * Hold the shoulder at the current angle
     *
     * @return void
     */
    public void stop() {
        rightElevatorMotor.setVoltage(0.0);
        leftElevatorMotor.setVoltage(0.0);
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
        log.dashboardVerbose("marginOfError", marginOfError);

        return withinMarginOfError;
    }

    /**
     * Returns true if the shoulder is at an angle where it can be stowed
     *
     * @return True if the shoulder is at an angle where it can be stowed
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
