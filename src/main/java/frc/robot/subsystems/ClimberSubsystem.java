package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
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
    private SparkMax climbMotor;
    private RelativeEncoder encoder;

    /**
    *
    */
    public ClimberSubsystem(SubsystemsConfig subsystemsConfig) {
        super(subsystemsConfig.climberSubsystem);
        if (checkDisabled()) {
            return;
        }

        // TODO: this should be using the LinearMotor class
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

        climbMotor = new SparkMax(config.climberMotorCanId, MotorType.kBrushless);
        sparkMaxConfig.inverted(false).voltageCompensation(12.0).idleMode(IdleMode.kBrake);
        climbMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = climbMotor.getEncoder();
    }

    @Override
    public void periodic(){
        log.dashboard("EncoderPosition", encoder.getPosition());
    }

    public void start(Double speed) {
        if (checkDisabled()) {
            return;
        }

        climbMotor.set(speed);
    }

    public void stop() {
        if (checkDisabled()) {
            return;
        }

        climbMotor.set(0);
    }

}
