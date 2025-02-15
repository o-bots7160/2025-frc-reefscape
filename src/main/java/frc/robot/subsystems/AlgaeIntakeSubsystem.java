package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.manipulator.AlgaeIntakeCommand;

/**
 *
 */
@Logged
public class AlgaeIntakeSubsystem extends ObotSubsystemBase {
    private SparkMax     motor      = new SparkMax(55, MotorType.kBrushless);

    private TimeOfFlight haveSensor = new TimeOfFlight(101);

    /**
    *
    */
    public AlgaeIntakeSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.inverted(false).voltageCompensation(12.0).idleMode(IdleMode.kBrake);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        haveSensor.setRangingMode(RangingMode.Short, 24);

        addChild("algae/haveSensor", haveSensor);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("algaeSense", haveSensor.getRange());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void setSpeed(double new_speed) {
        motor.set(new_speed);
    }

    public boolean haveItem() {
        return haveSensor.getRange() < 100.0; // TODO: get this number
    }

    public Command algaeIntakeCommand(boolean new_intake) {
        return new AlgaeIntakeCommand(this, new_intake);
    }
}