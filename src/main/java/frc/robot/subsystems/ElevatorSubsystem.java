package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ElevatorCommand;

/**
 *
 */
@Logged
public class ElevatorSubsystem extends ObotSubsystemBase {

    // kS, kG, kV, kA TODO: do we need these for loaded intakes?
    ElevatorFeedforward            feedforward = new ElevatorFeedforward(1.0, 1.0, 1.0, 1.0);

    private SparkMax               rightElevatorMotor;

    private SparkMax               leftElevatorMotor;

    private RelativeEncoder        encoder;

    private final double           min_target  = 0.0;

    private final double           max_target  = 3000.0;

    private final double           kDt         = 0.02;

    private final DigitalInput     home        = new DigitalInput(0);

    // TODO: Max speed/accel?
    private final TrapezoidProfile profile     = new TrapezoidProfile(new TrapezoidProfile.Constraints(5.0, 0.75));

    private TrapezoidProfile.State goal        = new TrapezoidProfile.State();

    private TrapezoidProfile.State setpoint    = new TrapezoidProfile.State();

    /**
    *
    */
    public ElevatorSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();

        rightElevatorMotor = new SparkMax(0, MotorType.kBrushless);
        config.inverted(false).voltageCompensation(12.0).idleMode(IdleMode.kBrake);
        config.encoder.positionConversionFactor(1.0) // TODO determine these
                .velocityConversionFactor(1.0);
        config.softLimit.forwardSoftLimit(max_target).forwardSoftLimitEnabled(true).reverseSoftLimit(min_target)
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
        setpoint = new TrapezoidProfile.State(encoder.getPosition(), encoder.getVelocity());
        setpoint = profile.calculate(kDt, setpoint, goal);

        rightElevatorMotor.setVoltage(feedforward.calculate(setpoint.position, setpoint.velocity));

        putDashboardNumberVerbose("manipulator/elevator", setpoint.position);
        putDashboardBooleanVerbose("homePos", home.get());

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void setTarget(double new_target) {
        double target = new_target;

        if (new_target < min_target) {
            target = min_target;
        } else if (new_target > max_target) {
            target = max_target;
        }
        goal = new TrapezoidProfile.State(target, 0.0);
    }

    public boolean atTarget() {
        double err = Math.abs(Math.toDegrees(setpoint.position - goal.position));
        return err < 1.0;
    }

    public Command stowCommand() {
        return new ElevatorCommand(this, 0);
    }

    public Command goToCommand(double position) {
        return new ElevatorCommand(this, position);
    }

}
