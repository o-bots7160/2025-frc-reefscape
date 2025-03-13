package frc.robot.helpers;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CommandFactory;
import frc.robot.config.AllianceLandmarkConfig;
import frc.robot.devices.ButtonBoardController;
import frc.robot.devices.ButtonBoardController.ButtonBoardButton;
import frc.robot.devices.GameController;
import frc.robot.subsystems.DriveBaseSubsystem;

/**
 * The TriggerBindings class is responsible for mapping controller inputs (buttons and triggers) to their corresponding robot commands. It initializes
 * and manages bindings between the game controller, button board controller, and various robot subsystems.
 */
public class TriggerBindings {

    // Controllers
    ///////////////////////////////////////////
    private final GameController             gameController        = new GameController(0);

    private final ButtonBoardController      buttonBoardController = new ButtonBoardController(1, 2, 3, 4);

    // State
    ///////////////////////////////////////////

    private boolean                          coralSelected;

    private double                           algaeLevel;

    private double                           algaeRotation;

    private Map<ButtonBoardButton, Runnable> buttonActions;

    private Pose2d                           algaeReefPose         = new Pose2d();

    private Pose2d                           coralReefPose         = new Pose2d();

    private Pose2d                           coralStationFacePose  = new Pose2d();

    private Pose2d                           coralStationPose      = new Pose2d();

    private String                           lastReefAlgaeHeightSelected;

    private String                           lastReefSelected;

    // Misc
    ///////////////////////////////////////////
    private AllianceLandmarkConfig           landmarks;

    private final Logger                     log                   = Logger.getInstance(this.getClass());

    private CommandFactory                   cf;

    private DriveBaseSubsystem               driveBaseSubsystem;

    public TriggerBindings(
            AllianceLandmarkConfig allianceLandmarkConfig,
            CommandFactory commandFactory,
            DriveBaseSubsystem driveBaseSubsystem) {
        this.landmarks          = allianceLandmarkConfig;
        this.cf                 = commandFactory;
        this.driveBaseSubsystem = driveBaseSubsystem;

        // set initial state and configure mappings
        this.coralSelected      = buttonBoardController.isPressed(ButtonBoardButton.Switch);
        this.buttonActions      = Map.ofEntries(
                Map.entry(ButtonBoardButton.A, () -> {
                                            lastReefSelected    = "A";
                                            lastReefAlgaeHeightSelected = "High";
                                            coralReefPose       = landmarks.reefZoneA;
                                            algaeReefPose       = landmarks.reefZoneAB;
                                            algaeLevel          = landmarks.algaeHigh;
                                            algaeRotation       = landmarks.algaeHighRotation;
                                        }),
                Map.entry(ButtonBoardButton.B, () -> {
                    lastReefSelected    = "B";
                    lastReefAlgaeHeightSelected = "High";
                    coralReefPose       = landmarks.reefZoneB;
                    algaeReefPose       = landmarks.reefZoneAB;
                    algaeLevel          = landmarks.algaeHigh;
                    algaeRotation       = landmarks.algaeHighRotation;
                }),
                Map.entry(ButtonBoardButton.C, () -> {
                    lastReefSelected    = "C";
                    lastReefAlgaeHeightSelected = "Low";
                    coralReefPose       = landmarks.reefZoneC;
                    algaeReefPose       = landmarks.reefZoneCD;
                    algaeLevel          = landmarks.algaeLow;
                    algaeRotation       = landmarks.algaeLowRotation;
                }),
                Map.entry(ButtonBoardButton.D, () -> {
                    lastReefSelected    = "D";
                    lastReefAlgaeHeightSelected = "Low";
                    coralReefPose       = landmarks.reefZoneD;
                    algaeReefPose       = landmarks.reefZoneCD;
                    algaeLevel          = landmarks.algaeLow;
                    algaeRotation       = landmarks.algaeLowRotation;
                }),
                Map.entry(ButtonBoardButton.E, () -> {
                    lastReefSelected    = "E";
                    lastReefAlgaeHeightSelected = "High";
                    coralReefPose       = landmarks.reefZoneE;
                    algaeReefPose       = landmarks.reefZoneEF;
                    algaeLevel          = landmarks.algaeHigh;
                    algaeRotation       = landmarks.algaeHighRotation;
                }),
                Map.entry(ButtonBoardButton.F, () -> {
                    lastReefSelected    = "F";
                    lastReefAlgaeHeightSelected = "High";
                    coralReefPose       = landmarks.reefZoneF;
                    algaeReefPose       = landmarks.reefZoneEF;
                    algaeLevel          = landmarks.algaeHigh;
                    algaeRotation       = landmarks.algaeHighRotation;
                }),
                Map.entry(ButtonBoardButton.G, () -> {
                    lastReefSelected    = "G";
                    lastReefAlgaeHeightSelected = "Low";
                    coralReefPose       = landmarks.reefZoneG;
                    algaeReefPose       = landmarks.reefZoneGH;
                    algaeLevel          = landmarks.algaeLow;
                    algaeRotation       = landmarks.algaeLowRotation;
                }),
                Map.entry(ButtonBoardButton.H, () -> {
                    lastReefSelected    = "H";
                    lastReefAlgaeHeightSelected = "Low";
                    coralReefPose       = landmarks.reefZoneH;
                    algaeReefPose       = landmarks.reefZoneGH;
                    algaeLevel          = landmarks.algaeLow;
                    algaeRotation       = landmarks.algaeLowRotation;
                }),
                Map.entry(ButtonBoardButton.I, () -> {
                    lastReefSelected = "I";
                    coralReefPose = landmarks.reefZoneI;
                    algaeReefPose = landmarks.reefZoneIJ;
                    algaeLevel = landmarks.algaeHigh;
                    algaeRotation = landmarks.algaeHighRotation;
                }),
                Map.entry(ButtonBoardButton.J, () -> {
                    lastReefSelected    = "J";
                    lastReefAlgaeHeightSelected = "High";
                    coralReefPose       = landmarks.reefZoneJ;
                    algaeReefPose       = landmarks.reefZoneIJ;
                    algaeLevel          = landmarks.algaeHigh;
                    algaeRotation       = landmarks.algaeHighRotation;
                }),
                Map.entry(ButtonBoardButton.K, () -> {
                    lastReefSelected    = "K";
                    lastReefAlgaeHeightSelected = "Low";
                    coralReefPose       = landmarks.reefZoneK;
                    algaeReefPose       = landmarks.reefZoneKL;
                    algaeLevel          = landmarks.algaeLow;
                    algaeRotation       = landmarks.algaeLowRotation;
                }),
                Map.entry(ButtonBoardButton.L, () -> {
                    lastReefSelected    = "L";
                    lastReefAlgaeHeightSelected = "Low";
                    coralReefPose       = landmarks.reefZoneL;
                    algaeReefPose       = landmarks.reefZoneKL;
                    algaeLevel          = landmarks.algaeLow;
                    algaeRotation       = landmarks.algaeLowRotation;
                }));

        boolean anyReefPositionSelected = false;
        for (Map.Entry<ButtonBoardButton, Runnable> entry : buttonActions.entrySet()) {
            if (buttonBoardController.isPressed(entry.getKey())) {
                entry.getValue().run();
                log.info("Setting default state of reef.");
                anyReefPositionSelected = true;
                break;
            }
        }
        if (!anyReefPositionSelected) {
            log.warning("No reef position automatically selected!");
            buttonActions.get(ButtonBoardButton.A).run();
            coralSelected = true;
        }

        configureBindings();
    }

    private Command createLevelSelectCommand(String level, double coralLevel, double coralLevelRotation) {
        // Should only need to generate this once as it's using suppliers for the values that are changing
        Command                placeCoralCommand = cf.createPlaceCoralCommand(lastReefSelected, level, () -> coralReefPose, () -> coralLevel,
                () -> coralLevelRotation);
        Command                takeAlgaeCommand  = cf.createTakeAlgaeCommand(lastReefSelected, lastReefAlgaeHeightSelected, () -> algaeReefPose,
                () -> algaeLevel,
                () -> algaeRotation);

        // Create mappings and select
        Map<Boolean, Command>  mapOfEntries      = Map.ofEntries(Map.entry(true, placeCoralCommand), Map.entry(false, takeAlgaeCommand));

        SelectCommand<Boolean> selectCommand     = new SelectCommand<>(mapOfEntries, this::isCoralSelected);

        return selectCommand;
    }

    private void configureBindings() {
        assignArbitraryTriggerBindings();
        assignButtonBoardBindings();
        assignGameControllerBindings();
    }

    private void assignArbitraryTriggerBindings() {
        log.verbose("Assigning arbitrary trigger bindings");

        // TODO: validate that there's no performance issue with this approach
        // TODO: is there a better way to do this beyond pulling in the
        // driveBaseSubsystem?
        new Trigger(() -> driveBaseSubsystem.getPose().getY() <= 4.0386).onTrue(cf.execute(() -> {
            coralStationFacePose = landmarks.coralStationLeftFace;
            coralStationPose     = landmarks.coralStationLeft;
        })).onFalse(cf.execute(() -> {
            coralStationFacePose = landmarks.coralStationRightFace;
            coralStationPose     = landmarks.coralStationRight;
        }));
    }

    private void assignGameControllerBindings() {
        log.verbose("Assigning game controller bindings");

        // TODO: is this the right spot for this?
        Command driveBaseDefaultCommand = cf.createDriveBaseMoveManualCommandField(
                () -> gameController.getRawAxis(1) * landmarks.joystickInversion * (1 - gameController.getRawAxis(3) + 0.001)
                        / (1 - gameController.getRawAxis(2) + 0.001),
                () -> gameController.getRawAxis(0) * landmarks.joystickInversion * (1 - gameController.getRawAxis(3) + 0.001)
                        / (1 - gameController.getRawAxis(2) + 0.001),
                () -> gameController.getRawAxis(4));

        cf.setDriveBaseDefaultCommand(driveBaseDefaultCommand);
        // Assigning Buttons of the controller
        // gameController.onButtonHold(GameController.GameControllerButton.A, cf.createDriveBaseMoveToCommand(coralReefPose));
        gameController.onButtonHold(GameController.GameControllerButton.A, cf.createRotateShoulderCommand(() -> 90.0));
        // gameController.onButtonHold(GameController.GameControllerButton.B, cf.createLockCommand());
        gameController.onButtonHold(GameController.GameControllerButton.B, cf.createRotateShoulderCommand(() -> -146.0));
        gameController.onButtonHold(GameController.GameControllerButton.X, cf.createMoveElevatorCommand(() -> 0.0));
        gameController.onButtonHold(GameController.GameControllerButton.Y, cf.createMoveElevatorCommand(() -> 150.0));
        gameController.onButtonHold(GameController.GameControllerButton.L1, cf.createIngestAlgaeCommand());
        // gameController.onButtonHold(GameController.GameControllerButton.L1, cf.createMoveToCoralLevel1Command());
        gameController.onButtonHold(GameController.GameControllerButton.R1, cf.createIngestCoralCommand());
        // gameController.onButtonHold(GameController.GameControllerButton.R1, cf.createMoveToCoralLevel4Command());
        gameController.onButtonHold(GameController.GameControllerButton.Back, cf.createTestLoggerCommand("Back held"));
        gameController.onButtonHold(GameController.GameControllerButton.Start, cf.createDriveBaseResetAngleCommand(0.0));
        // gameController.onButtonHold(GameController.GameControllerButton.Start, cf.createDriveBaseMoveToCommand(landmarks.reefFaceGH));
        gameController.onButtonHold(GameController.GameControllerButton.LStick, cf.createTestLoggerCommand("LStick held"));
        gameController.onButtonHold(GameController.GameControllerButton.RStick, cf.createTestLoggerCommand("RStick held"));

    }

    private void assignButtonBoardBindings() {
        log.verbose("Assigning button board bindings");

        buttonBoardController.onButtonHold(ButtonBoardButton.Switch, cf.createSwitchChangedCommand((b) -> coralSelected = b));
        buttonBoardController.onButtonHold(ButtonBoardButton.Travel, cf.createTravelCommand());
        buttonBoardController.onButtonHold(ButtonBoardButton.CoralStation, cf.createCoralStationCommand());
        buttonBoardController.onButtonHold(ButtonBoardButton.Net, cf.createNetCommand());
        buttonBoardController.onButtonHold(ButtonBoardButton.Processor, cf.createProcessorCommand());

        Map<Boolean, Command>  mapOfEjectEntries  = Map.ofEntries(
                Map.entry(true, cf.createEjectCoralCommand()),
                Map.entry(false, cf.createEjectAlgaeCommand()));

        SelectCommand<Boolean> selectEjectCommand = new SelectCommand<>(mapOfEjectEntries, this::isCoralSelected);
        buttonBoardController.onButtonHold(ButtonBoardButton.Eject, selectEjectCommand);

        // Climbing commands
        ///////////////////////////////////////////
        buttonBoardController.onButtonHold(ButtonBoardButton.ClimbUp, cf.createClimbUpCommand());
        buttonBoardController.onButtonHold(ButtonBoardButton.ClimbDown, cf.createClimbDownCommand());

        // Reef Position State Assignment
        ///////////////////////////////////////////

        buttonActions.forEach((button, action) -> buttonBoardController.onButtonPress(button, cf.execute(action)));

        // Level commands (changes action based on state)
        ///////////////////////////////////////////

        // Assign to Button Hold
        buttonBoardController.onButtonHold(ButtonBoardButton.L1, createLevelSelectCommand("1", landmarks.coralLevel1, landmarks.coralLevel1Rotation));
        buttonBoardController.onButtonHold(ButtonBoardButton.L2, createLevelSelectCommand("2", landmarks.coralLevel2, landmarks.coralLevel2Rotation));
        buttonBoardController.onButtonHold(ButtonBoardButton.L3, createLevelSelectCommand("3", landmarks.coralLevel3, landmarks.coralLevel3Rotation));
        buttonBoardController.onButtonHold(ButtonBoardButton.L4, createLevelSelectCommand("4", landmarks.coralLevel4, landmarks.coralLevel4Rotation));
    }

    private boolean isCoralSelected() {
        return coralSelected;
    }

}
