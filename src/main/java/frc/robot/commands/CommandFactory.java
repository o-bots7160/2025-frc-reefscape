package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climber.ClimbDownCommand;
import frc.robot.commands.climber.ClimbUpCommand;
import frc.robot.commands.drivebase.MoveManualCommandField;
import frc.robot.commands.drivebase.MoveToCommand;
import frc.robot.commands.drivebase.ResetAngleCommand;
import frc.robot.commands.elevator.ClearElevatorCommand;
import frc.robot.commands.elevator.MoveElevatorCommand;
import frc.robot.commands.manipulator.RotateShoulderCommand;
import frc.robot.commands.manipulator.algae.EjectAlgaeCommand;
import frc.robot.commands.manipulator.algae.IngestAlgaeCommand;
import frc.robot.commands.manipulator.algae.PrepareTakeAlgaeCommand;
import frc.robot.commands.manipulator.coral.EjectCoralCommand;
import frc.robot.commands.manipulator.coral.IngestCoralCommand;
import frc.robot.commands.manipulator.coral.PreparePlaceCoralCommand;
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

    public void updateAllianceLandmarkConfig(AllianceLandmarkConfig config) {
        allianceLandmarkConfig = config;
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

    // Game Controller Commands
    ///////////////////////////////////////////
    public Command createDriveBaseMoveManualCommandField(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
        return new MoveManualCommandField(driveBaseSubsystem, x, y, rotation);
    }

    public Command createDriveBaseMoveToCommand(Supplier<Pose2d> pose) {
        Command command = new MoveToCommand(driveBaseSubsystem, pose);

        return wrapCommandWithLogging("Move To " + pose.get().getX() + ", " + pose.get().getY() + ", " + pose.get().getRotation().getDegrees(),
                command);
    }

    public Command createDriveBaseResetAngleCommand(double angle) {
        Command command = new ResetAngleCommand(driveBaseSubsystem, angle);

        return wrapCommandWithLogging("Reset Angle To " + angle + " Degrees", command);
    }

    public Command createDriveBaseLockCommand() {
        // TODO: make a dedicated command for this
        Command command = new InstantCommand(() -> driveBaseSubsystem.lockSwerveDrivePose());

        return wrapCommandWithLogging("Lock Swerve Drive Pose", command);
    }

    // Button Board Controller Commands
    ///////////////////////////////////////////
    public Command createPreparePlaceCoralCommand(String level, Supplier<Double> coralLevelSupplier, Supplier<Double> coralLevelRotation) {
        Command command = new PreparePlaceCoralCommand(
                coralIntakeSubsystem, elevatorSubsystem, shoulderSubsystem,
                coralLevelSupplier, coralLevelRotation);
        return wrapCommandWithLogging("Prepare to Place Coral at Level " + level, command);
    }

    public Command createPrepareTakeAlgaeCommand(String level, Supplier<Double> algaeLevelSupplier, Supplier<Double> algaeRotationRotation) {
        Command command = new PrepareTakeAlgaeCommand(
                algaeIntakeSubsystem, elevatorSubsystem, shoulderSubsystem,
                algaeLevelSupplier, algaeRotationRotation);
        return wrapCommandWithLogging("Prepare Take Algae at Level " + level, command);
    }

    public Command createTravelCommand() {
        // TODO: make a dedicated command for this; also need to rotate shoulder and not go below clear
        Command command = Commands.sequence(new ClearElevatorCommand(elevatorSubsystem).unless(() -> elevatorSubsystem.isClear()),
                Commands.parallel(new RotateShoulderCommand(shoulderSubsystem, 0), new MoveElevatorCommand(elevatorSubsystem, 20)));
        return wrapCommandWithLogging("Set Travel Position", command);
    }

    public Command createCoralStationCommand() {
        Command command = Commands.sequence(
                // Make sure we're clear to move
                new ClearElevatorCommand(elevatorSubsystem),
                // Put elevator and shoulder in proper position
                Commands.parallel(new RotateShoulderCommand(shoulderSubsystem, () -> allianceLandmarkConfig.coralStationRotation),

                        new MoveElevatorCommand(elevatorSubsystem, () -> allianceLandmarkConfig.coralStationHeight)),
                // Turn on Intake until coral has been consumed
                new IngestCoralCommand(coralIntakeSubsystem));

        return wrapCommandWithLogging("Move to Coral Station", command);
    }

    public Command createNetCommand() {
        Command command = Commands.sequence(
                // Make sure we're clear to move
                new ClearElevatorCommand(elevatorSubsystem),
                // Put elevator and shoulder in proper position
                Commands.parallel(new RotateShoulderCommand(shoulderSubsystem, () -> allianceLandmarkConfig.netRotation),

                        new MoveElevatorCommand(elevatorSubsystem, () -> allianceLandmarkConfig.netHeight)));

        return wrapCommandWithLogging("Move to Net", command);
    }

    public Command createProcessorCommand() {
        Command command = Commands.sequence(
                // Make sure we're clear to move
                new ClearElevatorCommand(elevatorSubsystem),
                // Put elevator and shoulder in proper position
                Commands.parallel(new RotateShoulderCommand(shoulderSubsystem, () -> allianceLandmarkConfig.processorRotation),

                        new MoveElevatorCommand(elevatorSubsystem, () -> allianceLandmarkConfig.processorHeight)));

        return wrapCommandWithLogging("Move to Processor", command);
    }

    public Command createEjectCoralCommand() {
        Command command = new EjectCoralCommand(coralIntakeSubsystem);

        return wrapCommandWithLogging("Eject Coral", command);
    }

    public Command createEjectAlgaeCommand() {
        Command command = new EjectAlgaeCommand(algaeIntakeSubsystem);

        return wrapCommandWithLogging("Eject Algae", command);
    }

    public Command createIngestCoralCommand() {
        Command command = new IngestCoralCommand(coralIntakeSubsystem);

        return wrapCommandWithLogging("Ingest Coral", command);
    }

    public Command createIngestAlgaeCommand() {
        Command command = new IngestAlgaeCommand(algaeIntakeSubsystem);

        return wrapCommandWithLogging("Ingest Algae", command);
    }

    public Command createClimbUpCommand() {
        Command command = new ClimbUpCommand(climberSubsystem);

        return wrapCommandWithLogging("Climb Up", command);
    }

    public Command createClimbDownCommand() {
        Command command = new ClimbDownCommand(climberSubsystem);

        return wrapCommandWithLogging("Climb Down", command);
    }

    public Command createIngestLollipops() {
        SequentialCommandGroup command = new SequentialCommandGroup(createProcessorCommand(), createIngestAlgaeCommand());

        return wrapCommandWithLogging("Ingest Lollipops", command);
    }

    // Autons
    ///////////////////////////////////////////

    public SendableChooser<Supplier<Command>> createAutonChooser() {

        // Left side auton
        Supplier<Command>                  moveToReefIJAndPlaceLevelFourCommand = () -> Commands.sequence(
                // Move to Position
                Commands.parallel(createDriveBaseMoveToCommand(() -> allianceLandmarkConfig.reefZoneJ),
                    createTravelCommand()).withTimeout(2.75),
                createPreparePlaceCoralCommand("4", () -> allianceLandmarkConfig.coralLevel4,
                        () -> allianceLandmarkConfig.coralLevel4Rotation),
                createEjectCoralCommand(),
                Commands.parallel(createTravelCommand(),
                    createDriveBaseMoveToCommand(() -> allianceLandmarkConfig.coralStationLeft)).withTimeout(3.0),
                createCoralStationCommand().until(() -> coralIntakeSubsystem.hasItem()),
                Commands.parallel(createDriveBaseMoveToCommand(() -> allianceLandmarkConfig.reefZoneK),
                    createTravelCommand()).withTimeout(3.0),
                createPreparePlaceCoralCommand("4", () -> allianceLandmarkConfig.coralLevel4,
                        () -> allianceLandmarkConfig.coralLevel4Rotation),
                createEjectCoralCommand(),
                Commands.parallel(createTravelCommand(),
                    createDriveBaseMoveToCommand(() -> allianceLandmarkConfig.coralStationLeft)).withTimeout(3.0),
                createCoralStationCommand().until(() -> coralIntakeSubsystem.hasItem()),
                Commands.parallel(createDriveBaseMoveToCommand(() -> allianceLandmarkConfig.reefZoneL),
                    createTravelCommand()).withTimeout(3.0),
                createPreparePlaceCoralCommand("4", () -> allianceLandmarkConfig.coralLevel4,
                        () -> allianceLandmarkConfig.coralLevel4Rotation),
                createEjectCoralCommand(),
                createTravelCommand());

        // Middle auton
        Supplier<Command>                  moveToReefGHAndPlaceLevelFourCommand = () -> Commands.sequence(
                // Move to Position
                Commands.parallel(createDriveBaseMoveToCommand(() -> allianceLandmarkConfig.reefZoneH),
                    createTravelCommand()).withTimeout(2.75),
                createPreparePlaceCoralCommand("4", () -> allianceLandmarkConfig.coralLevel4,
                        () -> allianceLandmarkConfig.coralLevel4Rotation),
                createEjectCoralCommand(),
                Commands.parallel(createDriveBaseMoveToCommand(() -> allianceLandmarkConfig.reefFaceGH),
                    createPrepareTakeAlgaeCommand("Low", () -> allianceLandmarkConfig.algaeLow, 
                        () -> allianceLandmarkConfig.algaeLowRotation)).withTimeout(2.75),
                Commands.parallel(createDriveBaseMoveToCommand(() -> allianceLandmarkConfig.reefZoneGH),
                    createPrepareTakeAlgaeCommand("Low", () -> allianceLandmarkConfig.algaeLow, 
                        () -> allianceLandmarkConfig.algaeLowRotation)).until(() -> algaeIntakeSubsystem.hasItem()),
                Commands.parallel(createProcessorCommand(),
                    createDriveBaseMoveToCommand(() -> allianceLandmarkConfig.reefFaceGH))/*,
                createNetCommand(),
                createEjectAlgaeCommand(),
                createTravelCommand()*/);

        // Right side auton
        Supplier<Command>                  moveToReefEFAndPlaceLevelFourCommand = () -> Commands.sequence(
                // Move to Position
                Commands.parallel(createDriveBaseMoveToCommand(() -> allianceLandmarkConfig.reefZoneE),
                    createTravelCommand()).withTimeout(2.75),
                createPreparePlaceCoralCommand("4", () -> allianceLandmarkConfig.coralLevel4,
                        () -> allianceLandmarkConfig.coralLevel4Rotation),
                createEjectCoralCommand(),
                Commands.parallel(createTravelCommand(),
                    createDriveBaseMoveToCommand(() -> allianceLandmarkConfig.coralStationRight)).withTimeout(3.0),
                createCoralStationCommand().until(() -> coralIntakeSubsystem.hasItem()),
                Commands.parallel(createDriveBaseMoveToCommand(() -> allianceLandmarkConfig.reefZoneD),
                    createTravelCommand()).withTimeout(3.0),
                createPreparePlaceCoralCommand("4", () -> allianceLandmarkConfig.coralLevel4,
                        () -> allianceLandmarkConfig.coralLevel4Rotation),
                createEjectCoralCommand(),
                Commands.parallel(createTravelCommand(),
                    createDriveBaseMoveToCommand(() -> allianceLandmarkConfig.coralStationRight)).withTimeout(3.0),
                createCoralStationCommand().until(() -> coralIntakeSubsystem.hasItem()),
                Commands.parallel(createDriveBaseMoveToCommand(() -> allianceLandmarkConfig.reefZoneC),
                    createTravelCommand()).withTimeout(3.0),
                createPreparePlaceCoralCommand("4", () -> allianceLandmarkConfig.coralLevel4,
                        () -> allianceLandmarkConfig.coralLevel4Rotation),
                createEjectCoralCommand(),
                createTravelCommand());

        SendableChooser<Supplier<Command>> chooser                              = new SendableChooser<Supplier<Command>>();
        chooser.addOption("Left Side Move", moveToReefIJAndPlaceLevelFourCommand);
        chooser.addOption("Middle Move", moveToReefGHAndPlaceLevelFourCommand);
        chooser.addOption("Right Side Move", moveToReefEFAndPlaceLevelFourCommand);

        return chooser;
    }

    // Sys ID Commands
    ///////////////////////////////////////////
    public Command createElevatorSysIdCommand(double delay, double quasiTimeout, double dynamicTimeout) {
        Command command = elevatorSubsystem.generateSysIdCommand(delay, quasiTimeout, dynamicTimeout);

        return wrapCommandWithLogging("Elevator Sys Id", command);
    }

    public Command createShoulderSysIdCommand(double delay, double quasiTimeout, double dynamicTimeout) {
        Command command = shoulderSubsystem.generateSysIdCommand(delay, quasiTimeout, dynamicTimeout);

        return wrapCommandWithLogging("Shoulder Sys Id", command);
    }

    public Command createDriveBaseAngleMotorSysIdCommand() {
        Command command = driveBaseSubsystem.getAngleMotorTestCommand();

        return wrapCommandWithLogging("Angle Motor Sys Id", command);
    }

    public Command createDriveBaseDriveMotorSysIdCommand() {
        Command command = driveBaseSubsystem.getDriveMotorTestCommand();

        return wrapCommandWithLogging("Drive Motor Sys Id", command);
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
