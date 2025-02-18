package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.commands.ElevatorCommand;

/**
 *
 */
@Logged
public class ElevatorSubsystem extends ObotSubsystemBase {

    // kS, kG, kV, kA TODO: do we need these for loaded intakes?
    ElevatorFeedforward            feedforward    = new ElevatorFeedforward(1.0, 1.0, 1.0, 1.0);

    private SparkMax               rightElevatorMotor;

    private SparkMax               leftElevatorMotor;

    private RelativeEncoder        encoder;

    private final double           kDt            = 0.02;

    private final DigitalInput     home           = new DigitalInput(0);

    // TODO: Measure value on robot
    private final double           metersPerPulse = 0.2;

    // meters TODO: Measure value on robot
    private final double           clearHeight    = 0.5;

    private final double           minHeight      = 0.0;

    // meters TODO: Measure value on robot
    private final double           maxHeight      = 2.0;

    // meters TODO: Is 1cm close enough? acheivable?
    private final double           deadband       = 0.01;

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
    public ElevatorSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();

        rightElevatorMotor = new SparkMax(0, MotorType.kBrushless);
        config.inverted(false).voltageCompensation(12.0).idleMode(IdleMode.kBrake);
        config.encoder.positionConversionFactor(metersPerPulse).velocityConversionFactor(metersPerPulse);
        config.softLimit.forwardSoftLimit(maxHeight - 0.01).forwardSoftLimitEnabled(true).reverseSoftLimit(minHeight)
                .reverseSoftLimitEnabled(true);
        rightElevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder           = rightElevatorMotor.getEncoder();

        leftElevatorMotor = new SparkMax(6, MotorType.kBrushless);
        config.inverted(true).voltageCompensation(12.0).idleMode(IdleMode.kBrake).follow(7);
        leftElevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    @Override
    public void periodic() {
        TrapezoidProfile.State temp_goal = goal;

        // check if the rest of the manipulator is clear to stow. if
        // not, only allow the elevator to go down to the clearHeight
        if ((goal.position < clearHeight) && !clearToStow.getAsBoolean()) {
            temp_goal = new TrapezoidProfile.State(clearHeight, 0.0);
        }
        setpoint = new TrapezoidProfile.State(encoder.getPosition(), encoder.getVelocity());
        setpoint = profile.calculate(kDt, setpoint, temp_goal);

        setVoltage(feedforward.calculate(setpoint.velocity));

        if (!home.get()) // Home elevator if down limit switch is made
        {
            encoder.setPosition(0.0);
        }
        if (verbosity) {
            SmartDashboard.putNumber("elevator/target", goal.position);
            SmartDashboard.putNumber("elevator/position", encoder.getPosition());
            SmartDashboard.putNumber("elevator/velocity", encoder.getVelocity());
            SmartDashboard.putBoolean("elevator/reset", home.get());
        }
        SmartDashboard.putBoolean("elevator/at_target", atTarget());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void setTarget(double new_target) {
        double target = new_target;

        if (new_target < minHeight) {
            target = minHeight;
        } else if (new_target > maxHeight) {
            target = maxHeight;
        }
        goal = new TrapezoidProfile.State(target, 0.0);
    }

    /**
     * Checks if elevator is not too low to move manipulator
     *
     * @return true if elevator clear of stowing
     */
    public boolean isClear() {
        return encoder.getPosition() > clearHeight;
    }

    public boolean atTarget() {
        double err = Math.abs(setpoint.position - goal.position);
        return err < deadband;
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
    private void logActivity(SysIdRoutineLog log) {
        log.motor("elevator")
                .voltage(Units.Volts.of(rightElevatorMotor.getBusVoltage() * rightElevatorMotor.getAppliedOutput()))
                .linearPosition(Units.Meters.of(encoder.getPosition()))
                .linearVelocity(LinearVelocity.ofBaseUnits(encoder.getVelocity(), Units.MetersPerSecond));
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
