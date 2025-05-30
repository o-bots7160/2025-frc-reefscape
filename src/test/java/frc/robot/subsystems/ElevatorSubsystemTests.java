package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        CommandScheduler.getInstance().setPeriod(kDt);

        // Initialize config
        config                                        = new SubsystemsConfig();
        config.elevatorSubsystem                      = new ElevatorSubsystemConfig();
        config.elevatorSubsystem.minimumSetPoint      = 10.0;
        config.elevatorSubsystem.maximumSetPoint      = 100.0;
        config.elevatorSubsystem.clearedPosition      = 10;
        config.elevatorSubsystem.enabled              = true;
        config.elevatorSubsystem.enableDefaultCommand = true;
        config.elevatorSubsystem.verbose              = false;
        config.elevatorSubsystem.maximumAcceleration  = 1.00;
        config.elevatorSubsystem.maximumVelocity      = 10.0;
        config.elevatorSubsystem.setPointTolerance    = 0.1;
        config.elevatorSubsystem.stoppingTolerance    = 5.0;
        config.elevatorSubsystem.rightMotorCanId      = 1;
        config.elevatorSubsystem.leftMotorCanId       = 2;

        // Initialize subsystems and devices
        elevatorSubsystem                             = ElevatorSubsystemMock.getInstance(config);
        elevatorSubsystem.resetPositions();
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
        double targetSetpoint = 90.0;
        double timeToRun      = getTimeToRun(targetSetpoint);

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

        // Analyze
        //////////////////////////////////////////////////
        logTimeSlices("targetReachedAfterUninteruptedSeek");
    }

    @Test
    void targetInterruptedSeekDuringCommandStopsMoving() throws InterruptedException {
        // Arrange
        //////////////////////////////////////////////////
        double  targetSetpoint = 90.0;
        double  timeToRun      = getTimeToRun(targetSetpoint);
        double  halfTimeToRun  = timeToRun / 2;

        // Will be executed via a command and the scheduler
        Command command        = getInteruptableCommand(targetSetpoint);
        CommandScheduler.getInstance().schedule(command);

        // Act
        //////////////////////////////////////////////////
        for (int i = 0; i < timeToRun / kDt; i++) {
            CommandScheduler.getInstance().run();

            // Cancel the command half way through
            if (i == halfTimeToRun / kDt) {
                command.cancel();
            }
        }

        // Assert
        //////////////////////////////////////////////////
        double currentPosition = elevatorSubsystem.getCurrentPosition();
        assertTrue(elevatorSubsystem.isInterupted());
        // assertTrue(currentPosition > targetSetpoint / 2);
        assertTrue(currentPosition < targetSetpoint);

        // Analyze
        //////////////////////////////////////////////////
        logTimeSlices("targetInterruptedSeekDuringCommandStopsMoving");
    }

    @Test
    void targetInterruptedSeekDuringSecondaryCommandStopsMoving() throws InterruptedException {
        // Arrange
        //////////////////////////////////////////////////
        double  targetSetpointA = 90.0;
        double  targetSetpointB = 20.0;
        double  timeToRunA      = getTimeToRun(targetSetpointA);
        double  timeToRunB      = getTimeToRun(targetSetpointA - targetSetpointB);
        double  timeToRun       = timeToRunA + timeToRunB;
        double  breakTime       = timeToRunA + timeToRunB / 2;

        // Will be executed via a command and the scheduler
        Command commandA        = getInteruptableCommand(targetSetpointA);
        Command commandB        = getInteruptableCommand(targetSetpointB);
        Command sequenceCommand = Commands.sequence(commandA, commandB);
        CommandScheduler.getInstance().schedule(sequenceCommand);

        // Act
        //////////////////////////////////////////////////
        for (int i = 0; i < timeToRun / kDt; i++) {
            CommandScheduler.getInstance().run();

            // Cancel the command after the
            if (i == breakTime / kDt) {
                sequenceCommand.cancel();
            }
        }

        // Assert
        //////////////////////////////////////////////////
        double currentPosition = elevatorSubsystem.getCurrentPosition();
        assertTrue(elevatorSubsystem.isInterupted());
        assertTrue(currentPosition < targetSetpointA);
        // assertTrue(currentPosition > targetSetpointB);

        // Analyze
        //////////////////////////////////////////////////
        logTimeSlices("targetInterruptedSeekDuringSecondaryCommandStopsMoving");
    }

    private Command getInteruptableCommand(double targetSetpoint) {
        Command command = new FunctionalCommand(
                () -> elevatorSubsystem.setTarget(targetSetpoint),
                elevatorSubsystem::seekTarget,
                interrupted -> {
                    if (interrupted) {
                        elevatorSubsystem.interrupt();
                    } else {
                        elevatorSubsystem.stop();
                    }
                },
                elevatorSubsystem::atTarget,
                elevatorSubsystem);
        return command;
    }

    private double getTimeToRun(double targetSetpoint) {
        double maxAccel              = config.elevatorSubsystem.maximumAcceleration;
        double maxVel                = config.elevatorSubsystem.maximumVelocity;

        double timeToMaxVelocity     = maxVel / maxAccel;
        double distanceToMaxVelocity = 0.5 * maxAccel * Math.pow(timeToMaxVelocity, 2);

        double remainingDistance     = targetSetpoint - 2 * distanceToMaxVelocity;
        double timeAtMaxVelocity     = remainingDistance / maxVel;

        double expectedTimeToTarget  = 2 * timeToMaxVelocity + timeAtMaxVelocity;
        double timeToRun             = expectedTimeToTarget + 1.0;

        return timeToRun;
    }

    private void logTimeSlices(String testName) {
        List<double[]> timeSlices = elevatorSubsystem.getTimeSlices();

        try {
            // Will be under build/jin/release/logs
            java.nio.file.Path path = java.nio.file.Paths.get("logs/elevator-" + testName + ".csv");
            java.nio.file.Files.createDirectories(path.getParent());
            try (FileWriter writer = new FileWriter(path.toFile(), false)) {
                writer.write("Time Slice, Current Position, Current Velocity\n");
                for (double[] slice : timeSlices) {
                    writer.write(String.format("%.4f,%.4f,%.4f\n", slice[0], slice[1], slice[2]));
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
