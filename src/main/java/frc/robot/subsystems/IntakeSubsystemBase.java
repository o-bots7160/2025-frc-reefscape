package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.config.IntakeSubsystemConfigBase;

public abstract class IntakeSubsystemBase<TConfig extends IntakeSubsystemConfigBase> extends ObotSubsystemBase {
    protected TConfig      config;

    protected SparkMax     motor;

    protected TimeOfFlight haveSensor;

    public IntakeSubsystemBase() {
        config     = getConfig();

        motor      = new SparkMax(config.sparkMaxCanId, MotorType.kBrushless);
        haveSensor = new TimeOfFlight(config.timeOfFlightSensorCanId);

        var sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.inverted(false).voltageCompensation(12.0).idleMode(IdleMode.kBrake);

        motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        haveSensor.setRangingMode(RangingMode.Short, 24);

        addChild(className + "/HaveSensor", haveSensor);
    }

    /**
     * Sets the speed of the SparkMax motor
     * 
     * @param
     */
    public void setSpeed(double new_speed) {
        motor.set(new_speed);
    }

    /**
     * Determines if the intake has an item
     * 
     * @return
     */
    public boolean haveItem() {
        return haveSensor.getRange() < config.timeOfFlightSensorThreshold;
    }

    @Override
    public void periodic() {
        putDashboardNumber("haveSensor/Range", haveSensor.getRange());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
    }

    /**
     * Gets the relevant config object for this substystem (e.g., from the
     * SubsystemConfig)
     */
    protected abstract TConfig getConfig();

}
