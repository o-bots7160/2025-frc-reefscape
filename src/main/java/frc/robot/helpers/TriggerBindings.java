package frc.robot.helpers;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.commands.CommandFactory;
import frc.robot.config.AllianceLandmarkConfig;
import frc.robot.devices.GameController;
import frc.robot.subsystems.DriveBaseSubsystem;

/**
 * The TriggerBindings class is responsible for mapping controller inputs (buttons and triggers) to their corresponding robot commands. It initializes
 * and manages bindings between the game controller, button board controller, and various robot subsystems.
 */
public class TriggerBindings {

    // Controllers
    ///////////////////////////////////////////
    private final GameController   driveGameController  = new GameController(0);

    private final GameController   actionGameController = new GameController(1);

    // State
    ///////////////////////////////////////////

    private boolean                coralSelected        = true;

    private Pose2d                 coralReefPose        = new Pose2d();

    private String                 lastReefSelected;

    // Misc
    ///////////////////////////////////////////
    private AllianceLandmarkConfig allianceLandmarkConfig;

    private final Logger           log                  = Logger.getInstance(this.getClass());

    private CommandFactory         cf;

    private DriveBaseSubsystem     driveBaseSubsystem;

    public TriggerBindings(
            AllianceLandmarkConfig allianceLandmarkConfig,
            CommandFactory commandFactory,
            DriveBaseSubsystem driveBaseSubsystem) {
        this.allianceLandmarkConfig = allianceLandmarkConfig;
        this.cf                     = commandFactory;
        this.driveBaseSubsystem     = driveBaseSubsystem;

        configureBindings();
    }

    public void updateAllianceLandmarkConfig(AllianceLandmarkConfig config) {
        allianceLandmarkConfig = config;
    }

    private Command createLevelSelectCommand(String level, Supplier<Double> coralLevel, Supplier<Double> coralLevelRotation,
            Supplier<Double> algaeLevel, Supplier<Double> algaeRotation) {
        // Should only need to generate this once as it's using suppliers for the values that are changing
        Command                placeCoralCommand = cf.createPreparePlaceCoralCommand(level, coralLevel, coralLevelRotation);
        Command                takeAlgaeCommand  = cf.createPrepareTakeAlgaeCommand("n/a", algaeLevel, algaeRotation);

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

        // Mappings for Driving Game Controller
        /////////////////////////////////////////////////

        // Sticks and Triggers
        Command driveBaseDefaultCommand = cf.createDriveBaseMoveManualCommandField(
                () -> driveGameController.getRawAxis(1) * allianceLandmarkConfig.joystickInversion * (1 - driveGameController.getRawAxis(3) + 0.001)
                        / (1 - driveGameController.getRawAxis(2) + 0.001),
                () -> driveGameController.getRawAxis(0) * allianceLandmarkConfig.joystickInversion * (1 - driveGameController.getRawAxis(3) + 0.001)
                        / (1 - driveGameController.getRawAxis(2) + 0.001),
                () -> driveGameController.getRawAxis(4) * 1.25);

        cf.setDriveBaseDefaultCommand(driveBaseDefaultCommand);

        // Main Buttons
        driveGameController.onButtonHold(GameController.GameControllerButton.A, cf.createTravelCommand());
        driveGameController.onButtonHold(GameController.GameControllerButton.B, cf.createDriveBaseLockCommand());
        driveGameController.onButtonHold(GameController.GameControllerButton.X, cf.createMoveElevatorCommand(() -> 0.0));
        driveGameController.onButtonHold(GameController.GameControllerButton.Y, cf.createMoveElevatorCommand(() -> 150.0));

        // Bumpers
        driveGameController.onButtonHold(GameController.GameControllerButton.L1, cf.createClimbDownCommand());
        driveGameController.onButtonHold(GameController.GameControllerButton.R1, cf.createClimbUpCommand());

        // Others
        driveGameController.onButtonHold(GameController.GameControllerButton.Start, cf.createDriveBaseResetAngleCommand(0.0));
        driveGameController.onButtonHold(GameController.GameControllerButton.Back, cf.createElevatorSysIdCommand(4.00, 5.00, 2.00));

        // Mappings for Action Game Controller
        //////////////////////////////////////////////////

        // Main Buttons
        actionGameController.onButtonHold(GameController.GameControllerButton.A,
                createLevelSelectCommand("1", () -> allianceLandmarkConfig.coralLevel1, () -> allianceLandmarkConfig.coralLevel1Rotation,
                        () -> allianceLandmarkConfig.algaeLow, () -> allianceLandmarkConfig.algaeLowRotation));
        actionGameController.onButtonHold(GameController.GameControllerButton.B,
                createLevelSelectCommand("2", () -> allianceLandmarkConfig.coralLevel2, () -> allianceLandmarkConfig.coralLevel2Rotation,
                        () -> allianceLandmarkConfig.algaeLow, () -> allianceLandmarkConfig.algaeLowRotation));
        actionGameController.onButtonHold(GameController.GameControllerButton.X,
                createLevelSelectCommand("3", () -> allianceLandmarkConfig.coralLevel3, () -> allianceLandmarkConfig.coralLevel3Rotation,
                        () -> allianceLandmarkConfig.algaeHigh,
                        () -> allianceLandmarkConfig.algaeHighRotation));
        actionGameController.onButtonHold(GameController.GameControllerButton.Y,
                createLevelSelectCommand("4", () -> allianceLandmarkConfig.coralLevel4, () -> allianceLandmarkConfig.coralLevel4Rotation,
                        () -> allianceLandmarkConfig.algaeHigh,
                        () -> allianceLandmarkConfig.algaeHighRotation));

        // Bumpers and Stick Buttons
        actionGameController.onButtonHold(GameController.GameControllerButton.L1, createEjectSelectCommand());
        actionGameController.onButtonHold(GameController.GameControllerButton.R1, cf.createCoralStationCommand());
        actionGameController.onButtonHold(GameController.GameControllerButton.LStick, cf.createNetCommand());
        actionGameController.onButtonHold(GameController.GameControllerButton.RStick, cf.createProcessorCommand());

        // Others
        actionGameController.onButtonPress(GameController.GameControllerButton.Start, new InstantCommand(this::toggleCoralSelected));
        actionGameController.onButtonPress(GameController.GameControllerButton.Back, cf.createTravelCommand());

    }

    private void toggleCoralSelected() {
        coralSelected = !coralSelected;
        log.dashboard("Coral Selected", coralSelected);
    }

    private Boolean isCoralSelected() {
        return coralSelected;
    }

}
