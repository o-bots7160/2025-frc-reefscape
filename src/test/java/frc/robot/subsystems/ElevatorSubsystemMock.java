package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.config.SubsystemsConfig;

public class ElevatorSubsystemMock extends ElevatorSubsystem {
    private static SparkMaxSim             maxSim;

    private static SparkAbsoluteEncoderSim absoluteEncoderSim;

    private static ElevatorSubsystemMock   instance;

    public static ElevatorSubsystemMock getInstance(SubsystemsConfig config) {
        if (instance == null) {
            instance = new ElevatorSubsystemMock(config);
        }
        return instance;
    }

    private double         currentTimeSlice = 0.0;

    private boolean        isInteruptted    = false;

    private List<double[]> timeSlices       = new ArrayList<>();

    public ElevatorSubsystemMock(SubsystemsConfig config) {
        super(config);

        // Setting up mocks
        // https://docs.revrobotics.com/revlib/spark/sim/simulation-getting-started
        // create the DCMotor objects to specify the motor type
        DCMotor  maxGearbox = DCMotor.getNEO(1);

        // create the normal Spark MAX object
        SparkMax max        = motors.get(0).motor.getMotor();

        // create the Spark MAX sim object
        maxSim             = new SparkMaxSim(max, maxGearbox);
        absoluteEncoderSim = maxSim.getAbsoluteEncoderSim();

        Command defaultCommand = new FunctionalCommand(
                () -> {
                    log.debug("Default command initializing");
                },
                // We're going to seek the target if we're interrupted, otherwise nothing
                this::seekTarget,
                interrupted -> {
                    log.debug("Default command interrupted");
                    stop();
                },
                this::atTarget,
                this);
        setDefaultCommand(defaultCommand);
    }

    public double getTarget() {
        return goalState.position;
    }

    @Override
    public void setTarget(double target) {
        currentTimeSlice = 0.0;
        timeSlices.clear();
        isInteruptted = false;

        super.setTarget(target);
    }

    public void interrupt() {
        isInteruptted = true;
        super.setTarget(getCurrentPosition() + 5);
    }

    @Override
    public double seekTarget() {
        currentTimeSlice += kDt;

        maxSim.iterate(nextState.velocity, 12, kDt);
        absoluteEncoderSim.iterate(nextState.velocity, kDt);
        double calculatedVoltage = super.seekTarget();

        timeSlices.add(new double[] { currentTimeSlice, nextState.position, nextState.velocity });

        return calculatedVoltage;
    }

    public List<double[]> getTimeSlices() {
        return timeSlices;
    }

}
