package frc.robot.helpers;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimbDownCommand;
import frc.robot.commands.ClimbUpCommand;
import frc.robot.commands.PlaceCoralCommand;
import frc.robot.commands.SwitchChangedCommand;
import frc.robot.commands.TakeAlgaeCommand;
import frc.robot.config.AllianceLandmarkConfig;
import frc.robot.devices.ButtonBoardController;
import frc.robot.devices.ButtonBoardController.ButtonBoardButton;
import frc.robot.devices.GameController;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

/**
 * The TriggerBindings class is responsible for mapping controller inputs
 * (buttons and triggers) to their corresponding robot commands. It initializes
 * and manages bindings between the game controller, button board controller,
 * and various robot subsystems.
 * <p>
 * To initialize all bindings, call the {@link #init()} method after
 * constructing an instance of this class.
 */
public class TriggerBindings {

    // Controllers
    ///////////////////////////////////////////
    private final GameController        gameController        = new GameController(0);

    private final ButtonBoardController buttonBoardController = new ButtonBoardController(1, 2, 3, 4);

    // Subsystems
    ///////////////////////////////////////////

    private AlgaeIntakeSubsystem        algaeIntakeSubsystem;

    private ClimberSubsystem            climberSubsystem;

    private CoralIntakeSubsystem        coralIntakeSubsystem;

    private DriveBaseSubsystem          driveBaseSubsystem;

    private ElevatorSubsystem           elevatorSubsystem;

    private ShoulderSubsystem           shoulderSubsystem;

    // State
    ///////////////////////////////////////////

    private boolean                     switchUp              = false;

    private double                      algaeLevel            = 0.0;

    private Pose2d                      algaeReefPose         = new Pose2d();

    private Pose2d                      coralReefPose         = new Pose2d();

    private Pose2d                      coralStationFacePose  = new Pose2d();

    private Pose2d                      coralStationPose      = new Pose2d();

    // Misc
    ///////////////////////////////////////////
    private AllianceLandmarkConfig      landmarks;

    private final Logger                log                   = Logger.getInstance(this.getClass());

    public TriggerBindings(
            // Config
            ///////////////////////////////////////////
            AllianceLandmarkConfig allianceLandmarkConfig,
            // Subsystems
            ///////////////////////////////////////////
            DriveBaseSubsystem driveBaseSubsystem, ClimberSubsystem climberSubsystem,
            CoralIntakeSubsystem coralIntakeSubsystem, AlgaeIntakeSubsystem algaeIntakeSubsystem,
            ShoulderSubsystem shoulderSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.landmarks            = allianceLandmarkConfig;

        this.driveBaseSubsystem   = driveBaseSubsystem;
        this.climberSubsystem     = climberSubsystem;
        this.coralIntakeSubsystem = coralIntakeSubsystem;
        this.algaeIntakeSubsystem = algaeIntakeSubsystem;
        this.shoulderSubsystem    = shoulderSubsystem;
        this.elevatorSubsystem    = elevatorSubsystem;
    }

    /**
     * Initializes all the bindings for the triggers and buttons.
     */
    public void init(AllianceLandmarkConfig allianceLandmarkConfig) {
        this.landmarks = allianceLandmarkConfig;
        init();
    }

    /**
     * Initializes all the bindings for the triggers and buttons.
     */
    public void init() {
        assignArbitraryTriggerBindings();
        assignButtonBoardBindings();
        assignGameControllerBindings();
    }

    private void assignArbitraryTriggerBindings() {
        log.verbose("Assigning arbitrary trigger bindings");

        // TODO: validate that there's no performance issue with this approach
        new Trigger(() -> driveBaseSubsystem.getPose().getY() <= 4.0386).onTrue(new InstantCommand(() -> {
            coralStationFacePose = landmarks.coralStationLeftFace;
            coralStationPose     = landmarks.coralStationLeft;
        })).onFalse(new InstantCommand(() -> {
            coralStationFacePose = landmarks.coralStationRightFace;
            coralStationPose     = landmarks.coralStationRight;
        }));
    }

    private void assignGameControllerBindings() {
        log.verbose("Assigning game controller bindings");

        driveBaseSubsystem.setDefaultCommand(driveBaseSubsystem.moveManual(
                () -> gameController.getRawAxis(1) * landmarks.joystickInversion,
                () -> gameController.getRawAxis(0) * landmarks.joystickInversion, () -> gameController.getRawAxis(4)));
    }

    private void assignButtonBoardBindings() {
        log.verbose("Assigning button board bindings");

        // TODO: I don't know if this will work as expected; may need to adjust the
        // command
        buttonBoardController.onButtonHold(ButtonBoardButton.Switch, new SwitchChangedCommand((b) -> switchUp = b));

        // Climbing commands
        ///////////////////////////////////////////
        buttonBoardController.onButtonHold(ButtonBoardButton.ClimbUp, new ClimbUpCommand(climberSubsystem));
        buttonBoardController.onButtonHold(ButtonBoardButton.ClimbDown, new ClimbDownCommand(climberSubsystem));

        // Reef Position State Assignment
        ///////////////////////////////////////////
        buttonBoardController.onButtonPress(ButtonBoardButton.A, new InstantCommand(() -> {
            coralReefPose = landmarks.reefZoneA;
            algaeReefPose = landmarks.reefZoneAB;
            algaeLevel    = landmarks.algaeHigh;
        }));

        buttonBoardController.onButtonPress(ButtonBoardButton.B, new InstantCommand(() -> {
            coralReefPose = landmarks.reefZoneB;
            algaeReefPose = landmarks.reefZoneAB;
            algaeLevel    = landmarks.algaeHigh;
        }));

        buttonBoardController.onButtonPress(ButtonBoardButton.C, new InstantCommand(() -> {
            coralReefPose = landmarks.reefZoneC;
            algaeReefPose = landmarks.reefZoneCD;
            algaeLevel    = landmarks.algaeLow;
        }));
        buttonBoardController.onButtonPress(ButtonBoardButton.D, new InstantCommand(() -> {
            coralReefPose = landmarks.reefZoneD;
            algaeReefPose = landmarks.reefZoneCD;
            algaeLevel    = landmarks.algaeLow;
        }));
        buttonBoardController.onButtonPress(ButtonBoardButton.E, new InstantCommand(() -> {
            coralReefPose = landmarks.reefZoneE;
            algaeReefPose = landmarks.reefZoneEF;
            algaeLevel    = landmarks.algaeHigh;
        }));
        buttonBoardController.onButtonPress(ButtonBoardButton.F, new InstantCommand(() -> {
            coralReefPose = landmarks.reefZoneF;
            algaeReefPose = landmarks.reefZoneEF;
            algaeLevel    = landmarks.algaeHigh;
        }));
        buttonBoardController.onButtonPress(ButtonBoardButton.G, new InstantCommand(() -> {
            coralReefPose = landmarks.reefZoneG;
            algaeReefPose = landmarks.reefZoneGH;
            algaeLevel    = landmarks.algaeLow;
        }));
        buttonBoardController.onButtonPress(ButtonBoardButton.H, new InstantCommand(() -> {
            coralReefPose = landmarks.reefZoneH;
            algaeReefPose = landmarks.reefZoneGH;
            algaeLevel    = landmarks.algaeLow;
        }));
        buttonBoardController.onButtonPress(ButtonBoardButton.I, new InstantCommand(() -> {
            coralReefPose = landmarks.reefZoneI;
            algaeReefPose = landmarks.reefZoneIJ;
            algaeLevel    = landmarks.algaeHigh;
        }));
        buttonBoardController.onButtonPress(ButtonBoardButton.J, new InstantCommand(() -> {
            coralReefPose = landmarks.reefZoneJ;
            algaeReefPose = landmarks.reefZoneIJ;
            algaeLevel    = landmarks.algaeHigh;
        }));
        buttonBoardController.onButtonPress(ButtonBoardButton.K, new InstantCommand(() -> {
            coralReefPose = landmarks.reefZoneK;
            algaeReefPose = landmarks.reefZoneKL;
            algaeLevel    = landmarks.algaeLow;
        }));
        buttonBoardController.onButtonPress(ButtonBoardButton.L, new InstantCommand(() -> {
            coralReefPose = landmarks.reefZoneL;
            algaeReefPose = landmarks.reefZoneKL;
            algaeLevel    = landmarks.algaeLow;
        }));

        // Level commands (changes action based on state)
        ///////////////////////////////////////////

        // Assign to Button Hold
        buttonBoardController.onButtonHold(ButtonBoardButton.L1, createLevelSelectCommand(landmarks.coralLevel1));
        buttonBoardController.onButtonHold(ButtonBoardButton.L2, createLevelSelectCommand(landmarks.coralLevel2));
        buttonBoardController.onButtonHold(ButtonBoardButton.L3, createLevelSelectCommand(landmarks.coralLevel3));
        buttonBoardController.onButtonHold(ButtonBoardButton.L4, createLevelSelectCommand(landmarks.coralLevel4));

        // buttonBoardController.onButtonHold(ButtonBoardButton.Net,
        // new NetCommand(algaeIntakeSubsystem, elevatorSubsystem, shoulderSubsystem));
        // buttonBoardController.onButtonHold(ButtonBoardButton.Processor,
        // new PlaceProcessorCommand(driveBaseSubsystem, algaeIntakeSubsystem,
        // elevatorSubsystem,
        // shoulderSubsystem, landmarks.processorFace, landmarks.processor));
        // buttonBoardController.onButtonHold(ButtonBoardButton.CoralStation,
        // new CollectCoralCommand(driveBaseSubsystem, coralIntakeSubsystem,
        // elevatorSubsystem, shoulderSubsystem,
        // getCoralStationFacePose(), getCoralStationPose()));

    }

    private Command createLevelSelectCommand(double coralLevel) {
        // Should only need to generate this once as it's using suppliers for the values
        // that are changing
        PlaceCoralCommand      placeCoralCommand = new PlaceCoralCommand(
                // Subsystems
                driveBaseSubsystem, coralIntakeSubsystem, elevatorSubsystem, shoulderSubsystem,
                // Targets
                () -> algaeReefPose, () -> coralReefPose, () -> coralLevel);
        TakeAlgaeCommand       takeAlgaeCommand  = new TakeAlgaeCommand();

        // Create mappings and select
        Map<Boolean, Command>  mapOfEntries      = Map.ofEntries(Map.entry(true, placeCoralCommand),
                Map.entry(false, takeAlgaeCommand));

        SelectCommand<Boolean> selectCommand     = new SelectCommand<>(mapOfEntries, this::isCoralSelected);

        return selectCommand;
    }

    private boolean isCoralSelected() {
        return switchUp;
    }

}
