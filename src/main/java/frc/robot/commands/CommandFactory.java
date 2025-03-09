package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drivebase.MoveManualCommandField;
import frc.robot.commands.elevator.MoveElevatorCommand;
import frc.robot.commands.manipulator.coral.PlaceCoralCommand;
import frc.robot.commands.manipulator.shoulder.RotateShoulderCommand;
import frc.robot.config.AllianceLandmarkConfig;
import frc.robot.helpers.Logger;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

/**
 * CommandFactory is a class that provides methods to create commands for the robot.
 */
public class CommandFactory {
    private AlgaeIntakeSubsystem   algaeIntakeSubsystem;

    private ClimberSubsystem       climberSubsystem;

    private CoralIntakeSubsystem   coralIntakeSubsystem;

    private DriveBaseSubsystem     driveBaseSubsystem;

    private ElevatorSubsystem      elevatorSubsystem;

    private ShoulderSubsystem      shoulderSubsystem;

    private AllianceLandmarkConfig allianceLandmarkConfig;

    private Logger                 log = Logger.getInstance(this.getClass());

    public CommandFactory(AlgaeIntakeSubsystem algaeIntakeSubsystem, ClimberSubsystem climberSubsystem, CoralIntakeSubsystem coralIntakeSubsystem,
            DriveBaseSubsystem driveBaseSubsystem, ElevatorSubsystem elevatorSubsystem, ShoulderSubsystem shoulderSubsystem,
            AllianceLandmarkConfig allianceLandmarkConfig) {
        this.algaeIntakeSubsystem   = algaeIntakeSubsystem;
        this.climberSubsystem       = climberSubsystem;
        this.coralIntakeSubsystem   = coralIntakeSubsystem;
        this.driveBaseSubsystem     = driveBaseSubsystem;
        this.elevatorSubsystem      = elevatorSubsystem;
        this.shoulderSubsystem      = shoulderSubsystem;
        this.allianceLandmarkConfig = allianceLandmarkConfig;
    }

    // Generic Utility Commands
    ///////////////////////////////////////////
    public Command createSwitchChangedCommand(Consumer<Boolean> switchChangedAction) {
        return new SwitchChangedCommand(switchChangedAction);
    }

    public Command createTestLoggerCommand(String message) {
        return execute(() -> log.info(message));
    }

    public Command execute(Runnable toRun) {
        return new InstantCommand(toRun);
    }

    // Subsystem Specific Commands
    ///////////////////////////////////////////
    public Command createRotateShoulderCommand(Supplier<Double> target) {
        return new RotateShoulderCommand(shoulderSubsystem, target);
    }

    public Command createMoveElevatorCommand(Supplier<Double> target) {
        return new MoveElevatorCommand(elevatorSubsystem, target);
    }

    // Default Commands
    ///////////////////////////////////////////
    public void setDriveBaseDefaultCommand(Command command) {
        driveBaseSubsystem.setDefaultCommand(command);
    }

    // Autonomous Commands
    ///////////////////////////////////////////
    public SendableChooser<Command> getAutonomousChooser() {
        return driveBaseSubsystem.getAutonomousChooser();
    }

    // Game Controller Commands
    ///////////////////////////////////////////
    public Command createDriveBaseMoveManualCommandField(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
        return new MoveManualCommandField(driveBaseSubsystem, x, y, rotation);
    }

    // Button Board Controller Commands
    ///////////////////////////////////////////
    public Command createPlaceCoralCommand(Supplier<Pose2d> coralReefPoseSupplier, Supplier<Double> coralLevelSupplier,
            Supplier<Double> coralLevelRotation) {
        Command command = new PlaceCoralCommand(
                // Subsystems
                driveBaseSubsystem, coralIntakeSubsystem, elevatorSubsystem, shoulderSubsystem,
                // Values
                coralReefPoseSupplier, coralLevelSupplier, coralLevelRotation);
        return wrapCommandWithLogging("Place Coral", command);
    }

    public Command createTakeAlgaeCommand(Supplier<Pose2d> algaeReefPoseSupplier, Supplier<Double> algaeLevelSupplier,
            Supplier<Double> algaeRotationRotation) {
        Command command = new PlaceCoralCommand(
                // Subsystems
                driveBaseSubsystem, coralIntakeSubsystem, elevatorSubsystem, shoulderSubsystem,
                // Values
                algaeReefPoseSupplier, algaeLevelSupplier, algaeRotationRotation);
        return wrapCommandWithLogging("Take Algae", command);
    }

    // Utilities
    ///////////////////////////////////////////
    private Command wrapCommandWithLogging(String name, Command commandToRun) {
        return Commands.sequence(
                // Log which command was executed
                execute(() -> log.dashboard("Last Command Executed", name)),
                // Execute the actual command
                commandToRun,
                // Log that it was completed
                execute(() -> log.info("Command " + name + " finished.")));
    }

}
