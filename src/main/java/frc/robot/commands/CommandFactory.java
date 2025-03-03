package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drivebase.MoveAtAngle;
import frc.robot.commands.drivebase.MoveFacingCommand;
import frc.robot.commands.drivebase.MoveManualCommandField;
import frc.robot.commands.drivebase.MoveManualCommandRobot;
import frc.robot.commands.drivebase.MoveToCommand;
import frc.robot.commands.drivebase.StopCommand;
import frc.robot.commands.manipulator.AlgaeIntakeCommand;
import frc.robot.commands.manipulator.CoralIntakeCommand;
import frc.robot.commands.manipulator.ShoulderCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

/**
 * CommandFactory is a class that provides static methods to create commands for
 * the robot.
 */
public class CommandFactory {
    private AlgaeIntakeSubsystem algaeIntakeSubsystem;

    private ClimberSubsystem     climberSubsystem;

    private CoralIntakeSubsystem coralIntakeSubsystem;

    private DriveBaseSubsystem   driveBaseSubsystem;

    private ElevatorSubsystem    elevatorSubsystem;

    private ShoulderSubsystem    shoulderSubsystem;

    public CommandFactory(AlgaeIntakeSubsystem algaeIntakeSubsystem, ClimberSubsystem climberSubsystem,
            CoralIntakeSubsystem coralIntakeSubsystem, DriveBaseSubsystem driveBaseSubsystem,
            ElevatorSubsystem elevatorSubsystem, ShoulderSubsystem shoulderSubsystem) {
        this.algaeIntakeSubsystem = algaeIntakeSubsystem;
        this.climberSubsystem     = climberSubsystem;
        this.coralIntakeSubsystem = coralIntakeSubsystem;
        this.driveBaseSubsystem   = driveBaseSubsystem;
        this.elevatorSubsystem    = elevatorSubsystem;
        this.shoulderSubsystem    = shoulderSubsystem;
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
        return new CollectAlgae(driveBaseSubsystem, algaeIntakeSubsystem, elevatorSubsystem, shoulderSubsystem,
                faceTarget, algaeTarget);
    }

    public Command createCollectCoralCommand(Pose2d faceTarget, Pose2d coralTarget) {
        return new CollectCoralCommand(driveBaseSubsystem, coralIntakeSubsystem, elevatorSubsystem, shoulderSubsystem,
                faceTarget, coralTarget);
    }

    public Command createEjectCommand() {
        return new EjectCommand(coralIntakeSubsystem, algaeIntakeSubsystem, elevatorSubsystem, shoulderSubsystem);
    }

    public Command createEjectCoralCommand() {
        return new EjectCoralCommand(coralIntakeSubsystem, shoulderSubsystem);
    }

    public Command createElevatorCommand(Supplier<Double> target) {
        return new ElevatorCommand(elevatorSubsystem, target);
    }

    public Command createNetCommand() {
        return new NetCommand(algaeIntakeSubsystem, elevatorSubsystem, shoulderSubsystem);
    }

    public Command createPlaceCoralCommand(Supplier<Pose2d> faceTarget, Supplier<Pose2d> reefTarget,
            Supplier<Double> levelTarget) {
        return new PlaceCoralCommand(driveBaseSubsystem, coralIntakeSubsystem, elevatorSubsystem, shoulderSubsystem,
                faceTarget, reefTarget, levelTarget);
    }

    public Command createPlaceProcessorCommand(Pose2d faceTarget, Pose2d algaeTarget) {
        return new PlaceProcessorCommand(driveBaseSubsystem, algaeIntakeSubsystem, elevatorSubsystem, shoulderSubsystem,
                faceTarget, algaeTarget);
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
        return new TravelCommand(elevatorSubsystem, shoulderSubsystem);
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

    public Command createManipulatorShoulderCommand(double target) {
        return new ShoulderCommand(shoulderSubsystem, target);
    }

    public void setDriveBaseDefaultCommand(Command command) {
        driveBaseSubsystem.setDefaultCommand(command);
    }

}
