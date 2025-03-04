package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import frc.robot.config.ClimberSubsystemConfig;
import frc.robot.config.SubsystemsConfig;

/**
 *
 */
@Logged
public class ClimberSubsystem extends ObotSubsystemBase<ClimberSubsystemConfig> {
    private SparkMax footMotor;

    private SparkMax climbMotor;

    /**
    *
    */
    public ClimberSubsystem(SubsystemsConfig subsystemsConfig) {
        super(subsystemsConfig.climberSubsystem);

        // TODO: this should be using the LinearMotor class
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

        climbMotor = new SparkMax(config.climberMotorCanId, MotorType.kBrushless);
        sparkMaxConfig.inverted(false).voltageCompensation(12.0).idleMode(IdleMode.kBrake);
        climbMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        footMotor = new SparkMax(config.footMotorCanId, MotorType.kBrushless);
        sparkMaxConfig.inverted(false).voltageCompensation(12.0).idleMode(IdleMode.kBrake);
        footMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void start(Double speed) {
        climbMotor.set(speed);
    }

    public void stop() {
        climbMotor.set(0);
    }

}
