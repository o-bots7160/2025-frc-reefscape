package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.manipulator.AlgaeIntakeCommand;
import frc.robot.commands.manipulator.CoralIntakeCommand;

/**
 *
 */
@Logged
public class ManipulatorSubsystem extends ObotSubsystemBase {

    private SparkMax     coralMotor;

    private SparkMax     algaeMotor;

    private TimeOfFlight haveAlgaeSensor = new TimeOfFlight(102);

    // private DigitalInput haveCoralSensor;
    private TimeOfFlight haveCoralSensor = new TimeOfFlight(101);

    private boolean      hasAlgae;

    private boolean      hasCoral;

    /**
    *
    */
    public ManipulatorSubsystem() {
        SparkMaxConfig coralConfig = new SparkMaxConfig();
        SparkMaxConfig algaeConfig = new SparkMaxConfig();

        coralMotor = new SparkMax(54, MotorType.kBrushless);
        coralConfig.inverted(false).voltageCompensation(12.0).idleMode(IdleMode.kBrake);
        coralMotor.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        algaeMotor = new SparkMax(55, MotorType.kBrushless);
        algaeConfig.inverted(false).voltageCompensation(12.0).idleMode(IdleMode.kBrake);
        algaeMotor.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        addChild("HaveAlgaeSensor", haveAlgaeSensor);

        addChild("HaveCoralSensor", haveCoralSensor);

    }

    @Override
    public void periodic() {
        // set the value of has Algae
        hasAlgae = 100 <= haveAlgaeSensor.getRange();
        hasCoral = 90 <= haveCoralSensor.getRange();
        // set based on a thresh hold of X
        // if above X will set to true
        // if below X will set to false

        // This method will be called once per scheduler run
        putDashboardNumber("coralSense", haveCoralSensor.getRange());
        putDashboardNumber("algaeSense", haveAlgaeSensor.getRange());
        putDashboardBoolean("hasAlgae", hasAlgae);
        putDashboardBoolean("hasCoral", hasCoral);
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

    public Command algaeIntakeCommand(boolean new_intake) {
        return new AlgaeIntakeCommand(this, new_intake);
    }

    public void setCoral(double new_speed) {
        coralMotor.set(new_speed);
    }

    public boolean haveCoral() {
        return false;
    }

    public Command coralIntakeCommand(boolean new_intake) {
        return new CoralIntakeCommand(this, new_intake);
    }
}
