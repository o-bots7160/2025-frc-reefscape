package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.manipulator.AlgaeIntakeCommand;
import frc.robot.commands.manipulator.CoralIntakeCommand;

/**
 *
 */
@Logged
public class ManipulatorSubsystem extends SubsystemBase {
    private double           lastPosition = 0.0;

    private SparkMax     elbowMotor;
    private SparkAbsoluteEncoder absEncoder;
    private SparkMax     coralMotor;
    private SparkMax     algaeMotor;
    private DigitalInput haveAlgaeSensor;
   // private DigitalInput haveCoralSensor;
    private TimeOfFlight haveCoralSensor = new TimeOfFlight( 102 );


    /**
    *
    */
    public ManipulatorSubsystem() {
        SparkMaxConfig elbowConfig = new SparkMaxConfig();
        SparkMaxConfig coralConfig = new SparkMaxConfig();
        SparkMaxConfig algaeConfig = new SparkMaxConfig();

        elbowMotor = new SparkMax(53, MotorType.kBrushless);
        elbowConfig
            .inverted( false )
            .voltageCompensation( 12.0 )
            .idleMode(IdleMode.kBrake);
        elbowConfig.absoluteEncoder
           .inverted( false )
           .positionConversionFactor( Math.PI )
           .velocityConversionFactor( Math.PI )
           .zeroCentered( true ) // center output range: -0.5 to 0.5 rather than 0.0 to 1.0
           .zeroOffset( 0.0 )    // TODO: Calibrate this offset should be straight down?   
           .setSparkMaxDataPortConfig( ); // Aboslutely required for Absolute encoder.
        elbowConfig.softLimit
           .forwardSoftLimit( Math.PI / 2.0 )
           .forwardSoftLimitEnabled( true )
           .reverseSoftLimit( -Math.PI / 2.0 )
           .reverseSoftLimitEnabled( true );
        elbowMotor.configure(elbowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        absEncoder   = elbowMotor.getAbsoluteEncoder();
        lastPosition = absEncoder.getPosition();
   
        coralMotor = new SparkMax(54, MotorType.kBrushless);
        coralConfig
            .inverted( false )
            .voltageCompensation( 12.0 )
            .idleMode(IdleMode.kBrake);
        coralMotor.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        algaeMotor = new SparkMax(55, MotorType.kBrushless);
        algaeConfig
            .inverted( false )
            .voltageCompensation( 12.0 )
            .idleMode(IdleMode.kBrake);
        algaeMotor.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        haveAlgaeSensor = new DigitalInput(0);
        addChild("HaveAlgaeSensor", haveAlgaeSensor);

        addChild("HaveCoralSensor", haveCoralSensor);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        lastPosition = absEncoder.getPosition();
        SmartDashboard.putNumber("coralSense", haveCoralSensor.getRange());
        SmartDashboard.putNumber("absEncoder", lastPosition);
        
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setAlgae(double new_speed) {
        algaeMotor.set(new_speed);
    }

    public boolean haveAlgae() {
        return false;
    }

    public Command algaeIntakeCommand(boolean new_intake)
    {
        return new AlgaeIntakeCommand(this, new_intake);
    }

    public void setCoral(double new_speed) {
        coralMotor.set(new_speed);
    }

    public boolean haveCoral() {
        return false;
    }

    public Command coralIntakeCommand(boolean new_intake)
    {
        return new CoralIntakeCommand(this, new_intake);
    }

}
