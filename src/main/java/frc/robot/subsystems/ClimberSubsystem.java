package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;

/**
 *
 */
@Logged
public class ClimberSubsystem extends ObotSubsystemBase {
    private SparkMax footMotor;

    private SparkMax climbMotor;

    /**
    *
    */
    public ClimberSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();

        climbMotor = new SparkMax(50, MotorType.kBrushless);
        config.inverted(false).voltageCompensation(12.0).idleMode(IdleMode.kBrake);
        climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        footMotor = new SparkMax(51, MotorType.kBrushless);
        config.inverted(false).voltageCompensation(12.0).idleMode(IdleMode.kBrake);
        footMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}
