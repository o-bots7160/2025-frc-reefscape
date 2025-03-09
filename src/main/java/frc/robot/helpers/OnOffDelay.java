package frc.robot.helpers;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;

public class OnOffDelay {
    private final Timer     timer    = new Timer();

    private boolean         real     = false;

    private boolean         filtered = false;

    private double          on_delay;

    private double          off_delay;

    private BooleanSupplier rawValue;

    public OnOffDelay(double new_on_delay, double new_off_delay, BooleanSupplier new_rawValue) {
        on_delay  = new_on_delay;
        off_delay = new_off_delay;
        rawValue  = new_rawValue;
    }

    public void setOnOffDelay(double new_on_delay, double new_off_delay) {
        on_delay  = new_on_delay;
        off_delay = new_off_delay;
    }

    //
    // Using timer deterimine if a signal is supposed to be on or off
    // after applying the appropriate delays
    //
    //
    public boolean isOn() {
        boolean value = rawValue.getAsBoolean();
        if (real) {
            if (value) {
                if (timer.hasElapsed(on_delay)) {
                    filtered = true;
                    timer.stop();
                }
            } else {
                timer.reset();
                timer.start();
            }
        } else {
            if (!value) {
                if (timer.hasElapsed(off_delay)) {
                    filtered = false;
                    timer.stop();
                }
            } else {
                timer.reset();
                timer.start();
            }
        }
        real = value;
        return filtered;
    }
}