package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 *
 */
@Logged
public class ManipulatorSubsystem extends SubsystemBase {
    private SparkMax     elbowMotor;
    private SparkMax     coralMotor;
    private SparkMax     algaeMotor;
    private DigitalInput haveAlgaeSensor;
    private DigitalInput haveCoralSensor;

    /**
    *
    */
    public ManipulatorSubsystem() {
        elbowMotor = new SparkMax(1, MotorType.kBrushless);
        elbowMotor.setInverted(false);

        coralMotor = new SparkMax(2, MotorType.kBrushless);
        coralMotor.setInverted(false);

        algaeMotor = new SparkMax(3, MotorType.kBrushless);
        algaeMotor.setInverted(false);

        haveAlgaeSensor = new DigitalInput(0);
        addChild("HaveAlgaeSensor", haveAlgaeSensor);

        haveCoralSensor = new DigitalInput(1);
        addChild("HaveCoralSensor", haveCoralSensor);

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
