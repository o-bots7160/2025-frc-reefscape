package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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
import frc.robot.commands.manipulator.ShoulderCommand;

/**
 *
 */
@Logged
public class ShoulderSubsystem extends ObotSubsystemBase {

    private final double           min_target  = -Math.PI * 0.75;

    private final double           max_target  = Math.PI * 0.75;

    private final double           kDt         = 0.02;

    private SparkMax               shoulderMotor;

    private SparkAbsoluteEncoder   absEncoder;

    private RelativeEncoder        relEncoder;

    // TODO: max speed/accel?
    private final TrapezoidProfile profile     = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.5, 0.10));

    private TrapezoidProfile.State goal        = new TrapezoidProfile.State();

    private TrapezoidProfile.State setpoint    = new TrapezoidProfile.State(Math.toRadians(-90.0), 0.0);

    // kS, kG, kV, kA
    // TODO: do we need these for loaded intakes?
    // ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.0, 4.0, 0.0);
    SimpleMotorFeedforward         feedforward = new SimpleMotorFeedforward(0.0, 1.0, 0.0);

    public BooleanSupplier         clearToSpin = () -> {
                                                   return true;
                                               };

    /**
     * Construct a new Shoulder Subsustem
     */
    public ShoulderSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();

        shoulderMotor = new SparkMax(53, MotorType.kBrushless);
        config.inverted(false).voltageCompensation(12.0).idleMode(IdleMode.kBrake);
        config.absoluteEncoder.inverted(false).positionConversionFactor(2.0 * Math.PI)
                .velocityConversionFactor(2.0 * Math.PI).zeroCentered(true) // center output range: -0.5 to 0.5 rather
                                                                            // than 0.0 to 1.0
                .zeroOffset(0.0) // TODO: Calibrate this offset should be straight down?
                .setSparkMaxDataPortConfig(); // Apparently required... Whats it do? Nobody knows.
        config.softLimit.forwardSoftLimit(max_target).forwardSoftLimitEnabled(false).reverseSoftLimit(min_target)
                .reverseSoftLimitEnabled(false);
        // Can we create/move PID control on/to motor controller?
        // config.closedLoop
        // .pidf( 0.0001, 0.0, 0.0, 1.0 )
        // .iMaxAccum( 0.5 )
        // .iZone( 0.01 )
        // config.softLimit.feedbackSensor(
        // ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder );

        shoulderMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        absEncoder = shoulderMotor.getAbsoluteEncoder();
        relEncoder = shoulderMotor.getEncoder();
    }

    @Override
    public void periodic() {
        /*double calculatedVoltage = 0.0;
        setpoint = new TrapezoidProfile.State(absEncoder.getPosition(), absEncoder.getVelocity());
        setpoint = profile.calculate(kDt, setpoint, goal);
        logVerbose("setpoint position: " + setpoint.position);

        calculatedVoltage = feedforward.calculate(setpoint.position, setpoint.velocity);
        logVerbose("Calculated Voltage:" + calculatedVoltage);
        shoulderMotor.setVoltage(calculatedVoltage);

        atTarget();

        putDashboardNumberVerbose("shoulder/position", Math.toDegrees(setpoint.position));
        putDashboardNumberVerbose("shoulder/rel", relEncoder.getPosition());
        putDashboardNumberVerbose("shoulder/target", Math.toDegrees(goal.position));
        putDashboardNumberVerbose("shoulder/Voltage", shoulderMotor.getAppliedOutput() * shoulderMotor.getBusVoltage());*/
    }

    @Override
    public void simulationPeriodic() {
    }

    /**
     * Sets the target angle for the shoulder to seek
     *
     * @param degrees that the shoulder needs turn to
     * @return void
     */
    public void setTarget(double degrees) {
        double radians = Math.toRadians(degrees);
        logVerbose("Setting target to " + radians + "(" + degrees + ")");
        if (radians > max_target) {
            radians = max_target;
            logVerbose("Target too high, clamping to " + radians + "(" + degrees + ")");
        } else {
            radians = min_target;
            logVerbose("Target too low, clamping to " + radians + "(" + degrees + ")");
        }
        goal = new TrapezoidProfile.State(radians, 0.0);
        logVerbose("Target set to " + goal.position);
    }

    /**
     * Returns true if the shoulder is at the target angle
     *
     * @return True if the shoulder is at the target angle
     */
    public boolean atTarget() {
        var    diff        = setpoint.position - goal.position;
        var    deg         = Math.toDegrees(diff);
        double err         = Math.abs(deg);
        var    lessThanErr = err < 1.0;
        logVerbose("Difference: " + diff);
        logVerbose("Degrees: " + deg);
        logVerbose("Error: " + err);
        logError("Less than error threshold: " + lessThanErr);
        return lessThanErr;
    }

    /**
     * Returns true if the shoulder is at an angle where it can be stowed
     *
     * @return True if the shoulder is at an angle where it can be stowed
     */
    public boolean isStowed() {
        double degrees = Math.toDegrees(setpoint.position); // Could do this in radians... but do we really want to?
        return (degrees > -91.0) && (degrees < -89.0);
    }

    /**
     * Creates a new Shoulder command to move to the desired angle
     *
     * @param degrees that the shoulder needs turn to
     * @return Command to move the shoulder
     */
    public Command shoulderCommand(double degrees) {
        System.out.println("Creating shoulder command");
        return new ShoulderCommand(this, degrees);
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
    private void setVoltage(Voltage new_voltage) {
        shoulderMotor.setVoltage(new_voltage.baseUnitMagnitude());
    }

    /**
     * Logs elevator motor activity for SysId
     *
     * @param log used to collect data
     * @return void
     */
    private void logActivity(SysIdRoutineLog log) {
        log.motor("shoulder").voltage(Units.Volts.of(shoulderMotor.getBusVoltage() * shoulderMotor.getAppliedOutput()))
                .angularPosition(Units.Radians.of(absEncoder.getPosition()))
                .angularVelocity(Units.RadiansPerSecond.of(absEncoder.getVelocity()));
    }

    /**
     * Creates a SysIdRoutine for this subsystem
     *
     * @param config - The Sys Id routine runner
     * @return A command that can be mapped to a button or other trigger
     */
    private SysIdRoutine setSysIdRoutine(Config config) {
        return new SysIdRoutine(config,
                new SysIdRoutine.Mechanism((volts) -> this.setVoltage(volts), (log) -> this.logActivity(log), this));
    }
}
