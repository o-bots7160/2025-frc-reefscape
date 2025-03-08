package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.climber.ClimbDownCommand;
import frc.robot.commands.climber.ClimbUpCommand;
import frc.robot.commands.drivebase.MoveAtAngle;
import frc.robot.commands.drivebase.MoveFacingCommand;
import frc.robot.commands.drivebase.MoveManualCommandField;
import frc.robot.commands.drivebase.MoveManualCommandRobot;
import frc.robot.commands.drivebase.MoveToCommand;
import frc.robot.commands.drivebase.StopCommand;
import frc.robot.commands.elevator.ClearElevatorCommand;
import frc.robot.commands.elevator.MoveElevatorCommand;
import frc.robot.commands.elevator.MoveElevatorToRangeCommand;
import frc.robot.commands.manipulator.algae.AlgaeIntakeCommand;
import frc.robot.commands.manipulator.algae.CollectAlgae;
import frc.robot.commands.manipulator.algae.EjectAlgaeCommand;
import frc.robot.commands.manipulator.algae.NetCommand;
import frc.robot.commands.manipulator.algae.PlaceProcessorCommand;
import frc.robot.commands.manipulator.algae.TakeAlgaeCommand;
import frc.robot.commands.manipulator.coral.CollectCoralCommand;
import frc.robot.commands.manipulator.coral.CoralIntakeCommand;
import frc.robot.commands.manipulator.coral.EjectCoralCommand;
import frc.robot.commands.manipulator.coral.PlaceCoralCommand;
import frc.robot.commands.manipulator.shoulder.RotateShoulderCommand;
import frc.robot.commands.multisystem.MoveToCoralPositionCommand;
import frc.robot.commands.multisystem.PrepareForCoralEjectionCommand;
import frc.robot.config.AllianceLandmarkConfig;
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

    public Command execute(Runnable toRun) {
        return new InstantCommand(toRun);
    }

    public Command createAutonomousCommand() {
        return new AutonomousCommand(driveBaseSubsystem);
    }

    public Command createClimbDownCommand() {
        return new ClimbDownCommand(climberSubsystem);
    }

    public Command createClimbUpCommand() {
        return new ClimbUpCommand(climberSubsystem);
    }

    public Command createCollectAlgae(Pose2d faceTarget, Pose2d algaeTarget) {
        return new CollectAlgae(driveBaseSubsystem, algaeIntakeSubsystem, elevatorSubsystem, shoulderSubsystem, faceTarget, algaeTarget);
    }

    public Command createCollectCoralCommand(Pose2d faceTarget, Pose2d coralTarget) {
        return new CollectCoralCommand(driveBaseSubsystem, coralIntakeSubsystem, elevatorSubsystem, shoulderSubsystem, faceTarget, coralTarget);
    }

    public Command createEjectCoralCommand() {
        return new EjectCoralCommand(coralIntakeSubsystem);
    }

    public Command createMoveElevatorCommand(Supplier<Double> target) {
        return new MoveElevatorCommand(elevatorSubsystem, target);
    }

    public Command createMoveElevatorCommand(double target) {
        return createMoveElevatorCommand(() -> target);
    }

    public Command createClearElevatorCommand() {
        return new MoveElevatorToRangeCommand(elevatorSubsystem, elevatorSubsystem::setClear, elevatorSubsystem::isClear);
    }

    public Command createStowElevatorCommand() {
        return new MoveElevatorToRangeCommand(elevatorSubsystem, elevatorSubsystem::setStow, elevatorSubsystem::isStowed);
    }

    public Command createMoveToCoralLevel1Command() {
        return new MoveToCoralPositionCommand(
                new ClearElevatorCommand(elevatorSubsystem).unless(() -> elevatorSubsystem.isClear()),
                new MoveElevatorCommand(elevatorSubsystem, allianceLandmarkConfig.coralLevel1),
                new RotateShoulderCommand(shoulderSubsystem, allianceLandmarkConfig.coralLevel1Rotation));
    }

    public Command createMoveToCoralLevel4Command() {
        return new MoveToCoralPositionCommand(
                new ClearElevatorCommand(elevatorSubsystem).unless(() -> elevatorSubsystem.isClear()),
                new MoveElevatorCommand(elevatorSubsystem, allianceLandmarkConfig.coralLevel4),
                new RotateShoulderCommand(shoulderSubsystem, allianceLandmarkConfig.coralLevel4Rotation));
    }

    public Command createMoveElevatorToCoralLevel2Command() {
        return createMoveElevatorCommand(allianceLandmarkConfig.coralLevel2);
    }

    public Command createMoveElevatorToCoralLevel3Command() {
        return createMoveElevatorCommand(allianceLandmarkConfig.coralLevel3);
    }

    public Command createMoveElevatorToCoralLevel4Command() {
        return createMoveElevatorCommand(allianceLandmarkConfig.coralLevel4);
    }

    public Command createRotateShoulderToCoralLevel1Command() {
        return createRotateShoulderCommand(allianceLandmarkConfig.coralLevel1);
    }

    public Command createRotateShoulderToCoralLevel2Command() {
        return createRotateShoulderCommand(allianceLandmarkConfig.coralLevel2);
    }

    public Command createRotateShoulderToCoralLevel3Command() {
        return createRotateShoulderCommand(allianceLandmarkConfig.coralLevel3);
    }

    public Command createRotateShoulderToCoralLevel4Command() {
        return createRotateShoulderCommand(allianceLandmarkConfig.coralLevel4);
    }

    public Command createPrepareForCoralEjectionAtLevel1Command() {
        return new PrepareForCoralEjectionCommand(createMoveToCoralLevel1Command(), createRotateShoulderToCoralLevel1Command());
    }

    public Command createPrepareForCoralEjectionAtLevel2Command() {
        return new PrepareForCoralEjectionCommand(createMoveElevatorToCoralLevel2Command(), createRotateShoulderToCoralLevel2Command());
    }

    public Command createPrepareForCoralEjectionAtLevel3Command() {
        return new PrepareForCoralEjectionCommand(createMoveElevatorToCoralLevel3Command(), createRotateShoulderToCoralLevel3Command());
    }

    public Command createPrepareForCoralEjectionAtLevel4Command() {
        return new PrepareForCoralEjectionCommand(createMoveElevatorToCoralLevel3Command(), createRotateShoulderToCoralLevel4Command());
    }

    public Command createElevatorSysIdCommand(double delay, double quasiTimeout, double dynamicTimeout) {
        return elevatorSubsystem.generateSysIdCommand(delay, quasiTimeout, dynamicTimeout);
    }

    public Command createShoulderSysIdCommand(double delay, double quasiTimeout, double dynamicTimeout) {
        return shoulderSubsystem.generateSysIdCommand(delay, quasiTimeout, dynamicTimeout);
    }

    public Command createNetCommand() {
        return new NetCommand(algaeIntakeSubsystem, elevatorSubsystem, shoulderSubsystem);
    }

    public Command createEjectAlgaeCommand() {
        return new EjectAlgaeCommand(algaeIntakeSubsystem);
    }

    public Command createPlaceCoralCommand(Supplier<Pose2d> faceTarget, Supplier<Pose2d> reefTarget, Supplier<Double> levelTarget) {
        return new PlaceCoralCommand(driveBaseSubsystem, coralIntakeSubsystem, elevatorSubsystem, shoulderSubsystem, faceTarget, reefTarget,
                levelTarget);
    }

    public Command createPlaceProcessorCommand(Pose2d faceTarget, Pose2d algaeTarget) {
        return new PlaceProcessorCommand(driveBaseSubsystem, algaeIntakeSubsystem, elevatorSubsystem, shoulderSubsystem, faceTarget, algaeTarget);
    }

    public Command createSwitchChangedCommand(Consumer<Boolean> switchChangedAction) {
        return new SwitchChangedCommand(switchChangedAction);
    }

    public Command createTakeAlgaeCommand() {
        return new TakeAlgaeCommand();
    }

    public Command createTestLoggerCommand(String name) {
        return new TestLoggerCommand(name);
    }

    public Command createTravelCommand() {
        return new TravelCommand(elevatorSubsystem, shoulderSubsystem, createClearElevatorCommand());
    }

    public Command createDriveBaseMoveAtAngle(DoubleSupplier x, DoubleSupplier y, Rotation2d rotation) {
        return new MoveAtAngle(driveBaseSubsystem, x, y, rotation);
    }

    public Command createDriveBaseMoveFacingCommand(DoubleSupplier x, DoubleSupplier y, Translation2d translation) {
        return new MoveFacingCommand(driveBaseSubsystem, x, y, translation);
    }

    public Command createDriveBaseMoveManualCommandField(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
        return new MoveManualCommandField(driveBaseSubsystem, x, y, rotation);
    }

    public Command createDriveBaseMoveManualCommandRobot(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
        return new MoveManualCommandRobot(driveBaseSubsystem, x, y, rotation);
    }

    public Command createDriveBaseMoveToCommand(Pose2d pose) {
        return new MoveToCommand(driveBaseSubsystem, pose);
    }

    public Command createDriveBaseStopCommand() {
        return new StopCommand(driveBaseSubsystem);
    }

    public Command createManipulatorAlgaeIntakeCommand(boolean intake) {
        return new AlgaeIntakeCommand(algaeIntakeSubsystem, intake);
    }

    public Command createManipulatorCoralIntakeCommand(boolean intake) {
        return new CoralIntakeCommand(coralIntakeSubsystem, intake);
    }

    public Command createRotateShoulderCommand(double target) {
        return new RotateShoulderCommand(shoulderSubsystem, target);
    }

    public void setDriveBaseDefaultCommand(Command command) {
        driveBaseSubsystem.setDefaultCommand(command);
    }

    public SendableChooser<Command> getAutonomousChooser() {
        return driveBaseSubsystem.getAutonomousChooser();
    }

    public Command setElevatorConstantCommand(double volts) {
        return elevatorSubsystem.moveConstantCommand(volts);
    }
}
