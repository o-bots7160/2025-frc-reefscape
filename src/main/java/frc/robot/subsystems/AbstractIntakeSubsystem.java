package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import frc.robot.config.AbstractIntakeSubsystemConfig;
import frc.robot.devices.IntakeMotor;
import frc.robot.helpers.OnOffDelay;

public abstract class AbstractIntakeSubsystem<TConfig extends AbstractIntakeSubsystemConfig> extends AbstractSubsystem<TConfig> {

    protected IntakeMotor  motor;

    protected TimeOfFlight haveSensor;

    protected boolean      intakeHasItem = false;

    protected OnOffDelay   debounce;

    public AbstractIntakeSubsystem(TConfig subsystemConfig) {
        super(subsystemConfig);
        if (checkDisabled()) {
            return;
        }

        motor      = new IntakeMotor(className, config.motorCanId);
        haveSensor = new TimeOfFlight(config.timeOfFlightSensorCanId);
        haveSensor.setRangingMode(RangingMode.Short, 24);

        debounce = new OnOffDelay(config.onDelay, config.offDelay, () -> haveSensor.getRange() < config.timeOfFlightSensorThreshold);

        addChild(className + "/Motor", motor);
        addChild(className + "/HaveSensor", haveSensor);
    }

    /**
     * Sets the speed of the SparkMax motor
     * 
     * @param
     */
    public void setSpeed(double speed) {
        if (checkDisabled()) {
            return;
        }
        motor.setSpeed(speed);
    }

    public void stop() {
        setSpeed(0.0);
    }

    public void ingest() {
        setSpeed(config.intakeSpeed);
    }

    public void eject() {
        setSpeed(config.intakeSpeed * -1.0);
    }

    /**
     * Determines if the intake has an item
     * 
     * @return
     */
    public boolean hasItem() {
        if (checkDisabled()) {
            return false;
        }

        return intakeHasItem;
    }

    @Override
    public void periodic() {
        if (checkDisabled()) {
            return;
        }

        intakeHasItem = debounce.isOn();
        log.dashboard("haveItem", intakeHasItem);
        log.dashboardVerbose("haveSensor/Range", haveSensor.getRange());
    }

}
