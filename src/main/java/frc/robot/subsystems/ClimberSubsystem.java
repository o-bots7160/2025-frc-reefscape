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
    private SparkMax leftClimbMotor;

    private SparkMax rightClimbMotor;

    /**
    *
    */
    public ClimberSubsystem() {
        var sparkMaxConfig = new SparkMaxConfig();
        leftClimbMotor = new SparkMax(4, MotorType.kBrushless);
        sparkMaxConfig.inverted(false).voltageCompensation(12.0).idleMode(IdleMode.kBrake);
        leftClimbMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        sparkMaxConfig = new SparkMaxConfig();
        rightClimbMotor = new SparkMax(5, MotorType.kBrushless);
        sparkMaxConfig.inverted(false).voltageCompensation(12.0).idleMode(IdleMode.kBrake);
        // set to follow leftClimbMotor?
        rightClimbMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
