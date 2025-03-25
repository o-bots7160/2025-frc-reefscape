package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.robot.config.ElevatorSubsystemConfig;
import frc.robot.config.SubsystemsConfig;

public class ElevatorSubsystemTests {
    static final double   DELTA = 1e-2;

    ElevatorSubsystemMock elevatorSubsystem;

    SubsystemsConfig      config;

    double                kDt   = 0.02;

    @BeforeEach
    void setup() {
        // Initialize HAL with a timeout of 500ms and no error
        assert HAL.initialize(500, 0);

        // Initialize config
        config                                       = new SubsystemsConfig();
        config.elevatorSubsystem                     = new ElevatorSubsystemConfig();
        config.elevatorSubsystem.minimumSetPoint     = 10.0;
        config.elevatorSubsystem.maximumSetPoint     = 100.0;
        config.elevatorSubsystem.clearedPosition     = 10;
        config.elevatorSubsystem.enabled             = true;
        config.elevatorSubsystem.verbose             = true;
        config.elevatorSubsystem.maximumAcceleration = 1.00;
        config.elevatorSubsystem.maximumVelocity     = 10.0;
        config.elevatorSubsystem.rightMotorCanId     = 1;
        config.elevatorSubsystem.leftMotorCanId      = 2;

        // Initialize subsystems and devices
        elevatorSubsystem                            = ElevatorSubsystemMock.getInstance(config);
    }

    @AfterEach
    void shutdown() throws Exception {
        elevatorSubsystem = null;
    }

    @Test
    void targetUpdatedAfterSet() {
        // Arrange
        //////////////////////////////////////////////////
        double targetSetpoint = 50.0;

        // Act
        //////////////////////////////////////////////////
        elevatorSubsystem.setTarget(targetSetpoint);

        // Assert
        //////////////////////////////////////////////////
        assertEquals(targetSetpoint, elevatorSubsystem.getTarget(), DELTA);
    }

    @Test
    void requestedSetpointUnderMinimumRaisesToMinimum() {
        // Arrange
        //////////////////////////////////////////////////
        double targetSetpoint = 5.0;

        // Act
        //////////////////////////////////////////////////
        elevatorSubsystem.setTarget(targetSetpoint);

        // Assert
        //////////////////////////////////////////////////
        assertEquals(config.elevatorSubsystem.minimumSetPoint, elevatorSubsystem.getTarget(), DELTA);
    }

    @Test
    void requestedSetpointAboveMaximumLowersToMaximum() {
        // Arrange
        //////////////////////////////////////////////////
        double targetSetpoint = 500.0;

        // Act
        //////////////////////////////////////////////////
        elevatorSubsystem.setTarget(targetSetpoint);

        // Assert
        //////////////////////////////////////////////////
        assertEquals(config.elevatorSubsystem.maximumSetPoint, elevatorSubsystem.getTarget(), DELTA);
    }

    @Test
    void targetReachedAfterUninteruptedSeek() throws InterruptedException {
        // Arrange
        //////////////////////////////////////////////////

        // calculate a target and expected times
        double targetSetpoint        = 90.0;
        double maxAccel              = config.elevatorSubsystem.maximumAcceleration;
        double maxVel                = config.elevatorSubsystem.maximumVelocity;

        double timeToMaxVelocity     = maxVel / maxAccel;
        double distanceToMaxVelocity = 0.5 * maxAccel * Math.pow(timeToMaxVelocity, 2);

        double remainingDistance     = targetSetpoint - 2 * distanceToMaxVelocity;
        double timeAtMaxVelocity     = remainingDistance / maxVel;

        double expectedTimeToTarget  = 2 * timeToMaxVelocity + timeAtMaxVelocity;
        double timeToRun             = expectedTimeToTarget + 1.0;

        // Set the target
        elevatorSubsystem.setTarget(targetSetpoint);

        // Act
        //////////////////////////////////////////////////

        for (int i = 0; i < timeToRun / kDt; i++) {
            elevatorSubsystem.seekTarget();
        }

        // Assert
        //////////////////////////////////////////////////
        double currentPosition = elevatorSubsystem.getCurrentPosition();
        assertEquals(targetSetpoint, currentPosition, DELTA);
    }

}
