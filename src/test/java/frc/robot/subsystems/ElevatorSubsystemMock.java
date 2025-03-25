package frc.robot.subsystems;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
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

    }

    public double getTarget() {
        return goalState.position;
    }

    @Override
    public void seekTarget() {
        maxSim.iterate(nextState.velocity, 12, kDt);
        absoluteEncoderSim.iterate(nextState.velocity, kDt);
        super.seekTarget();
    }

}
