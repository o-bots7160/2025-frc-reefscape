package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 *
 */
@Logged
public class ClimberSubsystem extends SubsystemBase {
    private SparkMax leftClimbMotor;
    private SparkMax rightClimbMotor;

    /**
    *
    */
    public ClimberSubsystem() {
        leftClimbMotor = new SparkMax(4, MotorType.kBrushless);
        leftClimbMotor.setInverted(false);

        rightClimbMotor = new SparkMax(5, MotorType.kBrushless);
        rightClimbMotor.setInverted(false);

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
