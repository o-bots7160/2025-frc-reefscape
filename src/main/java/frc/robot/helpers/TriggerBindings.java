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
 * <p>
 * To initialize all bindings, call the {@link #init()} method after constructing an instance of this class.
 */
public class TriggerBindings {

    // Controllers
    ///////////////////////////////////////////
    private final GameController        gameController        = new GameController(0);

    private final ButtonBoardController buttonBoardController = new ButtonBoardController(1, 2, 3, 4);

    // State
    ///////////////////////////////////////////

    private boolean                     switchUp              = false;

    private double                      algaeLevel;

    private double                      algaeRotation;

    private Pose2d                      algaeReefPose         = new Pose2d();

    private Pose2d                      coralReefPose         = new Pose2d();

    private Pose2d                      coralStationFacePose  = new Pose2d();

    private Pose2d                      coralStationPose      = new Pose2d();

    // Misc
    ///////////////////////////////////////////
    private AllianceLandmarkConfig      landmarks;

    private final Logger                log                   = Logger.getInstance(this.getClass());

    private CommandFactory              cf;

    private DriveBaseSubsystem          driveBaseSubsystem;

    public TriggerBindings(
            // Config
            ///////////////////////////////////////////
            AllianceLandmarkConfig allianceLandmarkConfig,
            // Command Management
            ///////////////////////////////////////////
            CommandFactory commandFactory,
            // Subsystems
            ///////////////////////////////////////////
            DriveBaseSubsystem driveBaseSubsystem) {
        this.landmarks          = allianceLandmarkConfig;
        this.cf                 = commandFactory;
        this.driveBaseSubsystem = driveBaseSubsystem;

        configureBindings();
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
        Command driveBaseDefaultCommand = cf.createDriveBaseMoveManualCommandField(() -> gameController.getRawAxis(1) * landmarks.joystickInversion,
                () -> gameController.getRawAxis(0) * landmarks.joystickInversion, () -> gameController.getRawAxis(4));

        cf.setDriveBaseDefaultCommand(driveBaseDefaultCommand);
        // Assigning Buttons of the controller
        // gameController.onButtonHold(GameController.GameControllerButton.A, cf.createClimbUpCommand());
        gameController.onButtonHold(GameController.GameControllerButton.A, cf.createRotateShoulderCommand(() -> 90.0));
        // gameController.onButtonHold(GameController.GameControllerButton.B, cf.createLockCommand());
        gameController.onButtonHold(GameController.GameControllerButton.B, cf.createRotateShoulderCommand(() -> -146.0));
        gameController.onButtonHold(GameController.GameControllerButton.X, cf.createMoveElevatorCommand(() -> 0.0));
        gameController.onButtonHold(GameController.GameControllerButton.Y, cf.createMoveElevatorCommand(() -> 150.0));
        // gameController.onButtonHold(GameController.GameControllerButton.L1, cf.createTestLoggerCommand("L1 held"));
        // gameController.onButtonHold(GameController.GameControllerButton.L1, cf.createMoveToCoralLevel1Command());
        // gameController.onButtonHold(GameController.GameControllerButton.R1, cf.createTestLoggerCommand("R1 held"));
        // gameController.onButtonHold(GameController.GameControllerButton.R1, cf.createMoveToCoralLevel4Command());
        gameController.onButtonHold(GameController.GameControllerButton.Back, cf.createTestLoggerCommand("Back held"));
        gameController.onButtonHold(GameController.GameControllerButton.Start, cf.createTestLoggerCommand("Start held"));
        gameController.onButtonHold(GameController.GameControllerButton.LStick, cf.createTestLoggerCommand("LStick held"));
        gameController.onButtonHold(GameController.GameControllerButton.RStick, cf.createTestLoggerCommand("RStick held"));

    }

    private void assignButtonBoardBindings() {
        log.verbose("Assigning button board bindings");

        // TODO: I don't know if this will work as expected; may need to adjust the
        // command
        buttonBoardController.onButtonHold(ButtonBoardButton.Switch, cf.createSwitchChangedCommand((b) -> switchUp = b));
        buttonBoardController.onButtonHold(ButtonBoardButton.Travel, cf.createTravelCommand());
        buttonBoardController.onButtonHold(ButtonBoardButton.CoralStation, cf.createCoralStationCommand());
        buttonBoardController.onButtonHold(ButtonBoardButton.Net, cf.createNetCommand());
        buttonBoardController.onButtonHold(ButtonBoardButton.Processor, cf.createProcessorCommand());

        Map<Boolean, Command>  mapOfEjectEntries      = Map.ofEntries(
            Map.entry(true, cf.createEjectCoralCommand()),
            Map.entry(false, cf.createEjectAlgaeCommand())
        );

        SelectCommand<Boolean> selectEjectCommand     = new SelectCommand<>(mapOfEjectEntries, this::isCoralSelected);
        buttonBoardController.onButtonHold(ButtonBoardButton.Eject, selectEjectCommand); 

        // Climbing commands
        ///////////////////////////////////////////
        // buttonBoardController.onButtonHold(ButtonBoardButton.ClimbUp, cf.createClimbUpCommand());
        // buttonBoardController.onButtonHold(ButtonBoardButton.ClimbDown, cf.createClimbDownCommand());

        // Reef Position State Assignment
        ///////////////////////////////////////////
        buttonBoardController.onButtonPress(ButtonBoardButton.A, cf.execute(() -> {
            log.debug("Dial Changed to A");
            coralReefPose = landmarks.reefZoneA;
            algaeReefPose = landmarks.reefZoneAB;
            algaeLevel    = landmarks.algaeHigh;
            algaeRotation = landmarks.algaeHighRotation;
        }));

        buttonBoardController.onButtonPress(ButtonBoardButton.B, cf.execute(() -> {
            coralReefPose = landmarks.reefZoneB;
            algaeReefPose = landmarks.reefZoneAB;
            algaeLevel    = landmarks.algaeHigh;
            algaeRotation = landmarks.algaeHighRotation;
        }));

        buttonBoardController.onButtonPress(ButtonBoardButton.C, cf.execute(() -> {
            coralReefPose = landmarks.reefZoneC;
            algaeReefPose = landmarks.reefZoneCD;
            algaeLevel    = landmarks.algaeLow;
            algaeRotation = landmarks.algaeLowRotation;
        }));
        buttonBoardController.onButtonPress(ButtonBoardButton.D, cf.execute(() -> {
            coralReefPose = landmarks.reefZoneD;
            algaeReefPose = landmarks.reefZoneCD;
            algaeLevel    = landmarks.algaeLow;
            algaeRotation = landmarks.algaeLowRotation;
        }));
        buttonBoardController.onButtonPress(ButtonBoardButton.E, cf.execute(() -> {
            coralReefPose = landmarks.reefZoneE;
            algaeReefPose = landmarks.reefZoneEF;
            algaeLevel    = landmarks.algaeHigh;
            algaeRotation = landmarks.algaeHighRotation;
        }));
        buttonBoardController.onButtonPress(ButtonBoardButton.F, cf.execute(() -> {
            coralReefPose = landmarks.reefZoneF;
            algaeReefPose = landmarks.reefZoneEF;
            algaeLevel    = landmarks.algaeHigh;
            algaeRotation = landmarks.algaeHighRotation;
        }));
        buttonBoardController.onButtonPress(ButtonBoardButton.G, cf.execute(() -> {
            coralReefPose = landmarks.reefZoneG;
            algaeReefPose = landmarks.reefZoneGH;
            algaeLevel    = landmarks.algaeLow;
            algaeRotation = landmarks.algaeLowRotation;
        }));
        buttonBoardController.onButtonPress(ButtonBoardButton.H, cf.execute(() -> {
            coralReefPose = landmarks.reefZoneH;
            algaeReefPose = landmarks.reefZoneGH;
            algaeLevel    = landmarks.algaeLow;
            algaeRotation = landmarks.algaeLowRotation;
        }));
        buttonBoardController.onButtonPress(ButtonBoardButton.I, cf.execute(() -> {
            coralReefPose = landmarks.reefZoneI;
            algaeReefPose = landmarks.reefZoneIJ;
            algaeLevel    = landmarks.algaeHigh;
            algaeRotation = landmarks.algaeHighRotation;
        }));
        buttonBoardController.onButtonPress(ButtonBoardButton.J, cf.execute(() -> {
            coralReefPose = landmarks.reefZoneJ;
            algaeReefPose = landmarks.reefZoneIJ;
            algaeLevel    = landmarks.algaeHigh;
            algaeRotation = landmarks.algaeHighRotation;
        }));
        buttonBoardController.onButtonPress(ButtonBoardButton.K, cf.execute(() -> {
            coralReefPose = landmarks.reefZoneK;
            algaeReefPose = landmarks.reefZoneKL;
            algaeLevel    = landmarks.algaeLow;
            algaeRotation = landmarks.algaeLowRotation;
        }));
        buttonBoardController.onButtonPress(ButtonBoardButton.L, cf.execute(() -> {
            coralReefPose = landmarks.reefZoneL;
            algaeReefPose = landmarks.reefZoneKL;
            algaeLevel    = landmarks.algaeLow;
            algaeRotation = landmarks.algaeLowRotation;
        }));

        // Level commands (changes action based on state)
        ///////////////////////////////////////////

        // Assign to Button Hold
        buttonBoardController.onButtonHold(ButtonBoardButton.L1, createLevelSelectCommand(landmarks.coralLevel1, landmarks.coralLevel1Rotation));
        buttonBoardController.onButtonHold(ButtonBoardButton.L2, createLevelSelectCommand(landmarks.coralLevel2, landmarks.coralLevel2Rotation));
        buttonBoardController.onButtonHold(ButtonBoardButton.L3, createLevelSelectCommand(landmarks.coralLevel3, landmarks.coralLevel3Rotation));
        buttonBoardController.onButtonHold(ButtonBoardButton.L4, createLevelSelectCommand(landmarks.coralLevel4, landmarks.coralLevel4Rotation));
    }

    private Command createLevelSelectCommand(double coralLevel, double coralLevelRotation) {
        // Should only need to generate this once as it's using suppliers for the values that are changing
        Command                placeCoralCommand = cf.createPlaceCoralCommand(() -> coralReefPose, () -> coralLevel, () -> coralLevelRotation);
        Command                takeAlgaeCommand  = cf.createTakeAlgaeCommand(() -> algaeReefPose, () -> algaeLevel, () -> algaeRotation);

        // Create mappings and select
        Map<Boolean, Command>  mapOfEntries      = Map.ofEntries(Map.entry(true, placeCoralCommand), Map.entry(false, takeAlgaeCommand));

        SelectCommand<Boolean> selectCommand     = new SelectCommand<>(mapOfEntries, this::isCoralSelected);

        return selectCommand;
    }

    private boolean isCoralSelected() {
        return switchUp;
    }

}
