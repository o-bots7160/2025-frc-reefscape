package frc.robot.helpers;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.commands.CommandFactory;
import frc.robot.config.AllianceLandmarkConfig;
import frc.robot.devices.GameController;
import frc.robot.devices.GameController.GameControllerButton;
import frc.robot.subsystems.DriveBaseSubsystem;

/**
 * The TriggerBindings class is responsible for mapping controller inputs (buttons and triggers) to their corresponding robot commands. It initializes
 * and manages bindings between the game controller, button board controller, and various robot subsystems.
 */
public class TriggerBindings {

    // Controllers
    ///////////////////////////////////////////
    private final GameController   gameController       = new GameController(0);

    private final GameController   gameControllerBackup = new GameController(1);

    // private final ButtonBoardController buttonBoardController = new ButtonBoardController(1, 2, 3, 4);

    // State
    ///////////////////////////////////////////

    private boolean                coralSelected        = true;

    private Pose2d                 coralReefPose        = new Pose2d();

    private String                 lastReefSelected;

    // Misc
    ///////////////////////////////////////////
    private AllianceLandmarkConfig landmarks;

    private final Logger           log                  = Logger.getInstance(this.getClass());

    private CommandFactory         cf;

    private DriveBaseSubsystem     driveBaseSubsystem;

    public TriggerBindings(
            AllianceLandmarkConfig allianceLandmarkConfig,
            CommandFactory commandFactory,
            DriveBaseSubsystem driveBaseSubsystem) {
        this.landmarks          = allianceLandmarkConfig;
        this.cf                 = commandFactory;
        this.driveBaseSubsystem = driveBaseSubsystem;

        configureBindings();
    }

    private Command createLevelSelectCommand(String level, double coralLevel, double coralLevelRotation, double algaeLevel, double algaeRotation) {
        // Should only need to generate this once as it's using suppliers for the values that are changing
        Command                placeCoralCommand = cf.createPlaceCoralCommand("n/a", level, () -> coralReefPose, () -> coralLevel,
                () -> coralLevelRotation);
        Command                takeAlgaeCommand  = cf.createTakeAlgaeCommand("n/a", "n/a", () -> new Pose2d(),
                () -> algaeLevel,
                () -> algaeRotation);

        // Create mappings and select
        Map<Boolean, Command>  mapOfEntries      = Map.ofEntries(Map.entry(true, placeCoralCommand), Map.entry(false, takeAlgaeCommand));

        SelectCommand<Boolean> selectCommand     = new SelectCommand<>(mapOfEntries, this::isCoralSelected);

        return selectCommand;
    }

    private Command createEjectSelectCommand() {
        Command                ejectCoral    = cf.createEjectCoralCommand();
        Command                ejectAlgae    = cf.createEjectAlgaeCommand();

        Map<Boolean, Command>  mapOfEntries  = Map.ofEntries(
                Map.entry(true, ejectCoral),
                Map.entry(false, ejectAlgae));
        SelectCommand<Boolean> selectCommand = new SelectCommand<>(mapOfEntries, this::isCoralSelected);

        return selectCommand;
    }

    private void configureBindings() {
        assignArbitraryTriggerBindings();
        assignGameControllerBindings();
    }

    private void assignArbitraryTriggerBindings() {
        log.verbose("Assigning arbitrary trigger bindings");

    }

    private void assignGameControllerBindings() {
        log.verbose("Assigning game controller bindings");

        // TODO: is this the right spot for this?
        Command driveBaseDefaultCommand = cf.createDriveBaseMoveManualCommandField(
                () -> gameController.getRawAxis(1) * landmarks.joystickInversion * (1 - gameController.getRawAxis(3) + 0.001)
                        / (1 - gameController.getRawAxis(2) + 0.001),
                () -> gameController.getRawAxis(0) * landmarks.joystickInversion * (1 - gameController.getRawAxis(3) + 0.001)
                        / (1 - gameController.getRawAxis(2) + 0.001),
                () -> gameController.getRawAxis(4) * 1.25);

        cf.setDriveBaseDefaultCommand(driveBaseDefaultCommand);
        // Assigning Buttons of the controller
        // gameController.onButtonHold(GameController.GameControllerButton.A, cf.createDriveBaseMoveToCommand(coralReefPose));
        gameController.onButtonHold(GameController.GameControllerButton.A, cf.createRotateShoulderCommand(() -> 90.0));
        // gameController.onButtonHold(GameController.GameControllerButton.B, cf.createLockCommand());
        gameController.onButtonHold(GameController.GameControllerButton.B, cf.createRotateShoulderCommand(() -> -146.0));
        gameController.onButtonHold(GameController.GameControllerButton.X, cf.createMoveElevatorCommand(() -> 0.0));
        gameController.onButtonHold(GameController.GameControllerButton.Y, cf.createMoveElevatorCommand(() -> 150.0));
        //gameController.onButtonHold(GameController.GameControllerButton.L1, cf.createIngestAlgaeCommand());
        // gameController.onButtonHold(GameController.GameControllerButton.L1, cf.createMoveToCoralLevel1Command());
        //gameController.onButtonHold(GameController.GameControllerButton.R1, cf.createIngestCoralCommand());
        // gameController.onButtonHold(GameController.GameControllerButton.R1, cf.createMoveToCoralLevel4Command());
        gameController.onButtonHold(GameController.GameControllerButton.Back, cf.createTestLoggerCommand("Back held"));
        gameController.onButtonHold(GameController.GameControllerButton.Start, cf.createDriveBaseResetAngleCommand(0.0));
        // gameController.onButtonHold(GameController.GameControllerButton.Start, cf.createDriveBaseMoveToCommand(landmarks.reefFaceGH));
        gameController.onButtonHold(GameController.GameControllerButton.LStick, cf.createTestLoggerCommand("LStick held"));
        gameController.onButtonHold(GameController.GameControllerButton.RStick, cf.createTestLoggerCommand("RStick held"));

        // gameController.onButtonHold(GameController.GameControllerButton.L1, () -> isCoralSelected = true, () -> isCoralSelected = false);
        gameControllerBackup.onButtonHold(GameController.GameControllerButton.R1, cf.createCoralStationCommand());
        gameControllerBackup.onButtonPress(GameController.GameControllerButton.Start, new InstantCommand(this::toggleCoralSelected));
        gameControllerBackup.onButtonHold(GameController.GameControllerButton.A,
                createLevelSelectCommand("1", landmarks.coralLevel1, landmarks.coralLevel1Rotation, landmarks.algaeLow, landmarks.algaeLowRotation));
        gameControllerBackup.onButtonHold(GameController.GameControllerButton.B,
                createLevelSelectCommand("2", landmarks.coralLevel2, landmarks.coralLevel2Rotation, landmarks.algaeLow, landmarks.algaeLowRotation));
        gameControllerBackup.onButtonHold(GameController.GameControllerButton.X,
                createLevelSelectCommand("3", landmarks.coralLevel3, landmarks.coralLevel3Rotation, landmarks.algaeHigh,
                        landmarks.algaeHighRotation));
        gameControllerBackup.onButtonHold(GameController.GameControllerButton.Y,
                createLevelSelectCommand("4", landmarks.coralLevel4, landmarks.coralLevel4Rotation, landmarks.algaeHigh,
                        landmarks.algaeHighRotation));
        gameControllerBackup.onButtonHold(GameController.GameControllerButton.L1, createEjectSelectCommand());
        gameControllerBackup.onButtonHold(GameController.GameControllerButton.LStick, cf.createNetCommand());
        gameControllerBackup.onButtonHold(GameController.GameControllerButton.RStick, cf.createProcessorCommand());

        gameController.onButtonHold(GameControllerButton.L1, cf.createClimbDownCommand());
        gameController.onButtonHold(GameControllerButton.R1, cf.createClimbUpCommand());
    }

    private void toggleCoralSelected() {
        coralSelected = !coralSelected;
        log.dashboard("Coral Selected", coralSelected);
    }

    private Boolean isCoralSelected() {
        return coralSelected;
    }

}
