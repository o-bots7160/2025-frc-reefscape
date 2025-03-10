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
import frc.robot.commands.elevator.ClearElevatorCommand;
import frc.robot.commands.elevator.MoveElevatorCommand;
import frc.robot.commands.manipulator.RotateShoulderCommand;
import frc.robot.commands.manipulator.algae.EjectAlgaeCommand;
import frc.robot.commands.manipulator.algae.IngestAlgaeCommand;
import frc.robot.commands.manipulator.algae.TakeAlgaeCommand;
import frc.robot.commands.manipulator.coral.EjectCoralCommand;
import frc.robot.commands.manipulator.coral.IngestCoralCommand;
import frc.robot.commands.manipulator.coral.PlaceCoralCommand;
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

    private Logger                 log = Logger.getInstance(this.getClass(), true);

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
        Command command = new TakeAlgaeCommand(
                // Subsystems
                driveBaseSubsystem, algaeIntakeSubsystem, elevatorSubsystem, shoulderSubsystem,
                // Values
                algaeReefPoseSupplier, algaeLevelSupplier, algaeRotationRotation);
        return wrapCommandWithLogging("Take Algae", command);
    }

    public Command createTravelCommand() {
        // TODO: make a dedicated command for this; also need to rotate shoulder and not go below clear
        Command command = Commands.sequence(new ClearElevatorCommand(elevatorSubsystem),
                Commands.parallel(new RotateShoulderCommand(shoulderSubsystem, 0), new MoveElevatorCommand(elevatorSubsystem, 20)));
        return wrapCommandWithLogging("Set Travel Position", command);
    }

    public Command createCoralStationCommand() {
        Command command = Commands.sequence(
                // Make sure we're clear to move
                new ClearElevatorCommand(elevatorSubsystem),
                // Put elevator and shoulder in proper position
                Commands.parallel(new RotateShoulderCommand(shoulderSubsystem, allianceLandmarkConfig.coralStationRotation),

                        new MoveElevatorCommand(elevatorSubsystem, allianceLandmarkConfig.coralStationHeight)),
                // Turn on Intake until coral has been consumed
                new IngestCoralCommand(coralIntakeSubsystem));

        return wrapCommandWithLogging("Move to Coral Station", command);
    }
    
    public Command createNetCommand() {
        Command command = Commands.sequence(
                // Make sure we're clear to move
                new ClearElevatorCommand(elevatorSubsystem),
                // Put elevator and shoulder in proper position
                Commands.parallel(new RotateShoulderCommand(shoulderSubsystem, allianceLandmarkConfig.netRotation),

                        new MoveElevatorCommand(elevatorSubsystem, allianceLandmarkConfig.netHeight)));

        return wrapCommandWithLogging("Move to Net", command);
    }
    
    public Command createProcessorCommand() {
        Command command = Commands.sequence(
                // Make sure we're clear to move
                new ClearElevatorCommand(elevatorSubsystem),
                // Put elevator and shoulder in proper position
                Commands.parallel(new RotateShoulderCommand(shoulderSubsystem, allianceLandmarkConfig.processorRotation),

                        new MoveElevatorCommand(elevatorSubsystem, allianceLandmarkConfig.processorHeight)));

        return wrapCommandWithLogging("Move to Processor", command);
    }

    public Command createEjectCoralCommand() {
        Command command = Commands.sequence(new EjectCoralCommand(coralIntakeSubsystem));

        return wrapCommandWithLogging("Eject Coral", command);
    }

    public Command createEjectAlgaeCommand() {
        Command command = Commands.sequence(new EjectAlgaeCommand(algaeIntakeSubsystem));

        return wrapCommandWithLogging("Eject Algae", command);
    }
    
    public Command createIngestCoralCommand() {
        Command command = Commands.sequence(new IngestCoralCommand(coralIntakeSubsystem));

        return wrapCommandWithLogging("Ingest Coral", command);
    }

    public Command createIngestAlgaeCommand() {
        Command command = Commands.sequence(new IngestAlgaeCommand(algaeIntakeSubsystem));

        return wrapCommandWithLogging("Ingest Algae", command);
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
