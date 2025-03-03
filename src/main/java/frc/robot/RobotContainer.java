package frc.robot;

import javax.naming.ConfigurationException;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.CommandFactory;
import frc.robot.config.AllianceLandmarksConfig;
import frc.robot.config.ConfigurationLoader;
import frc.robot.config.SubsystemsConfig;
import frc.robot.helpers.Logger;
import frc.robot.helpers.TriggerBindings;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
// @Logged
public class RobotContainer {

    private static RobotContainer robotContainer = new RobotContainer();

    public static RobotContainer getInstance() {
        return robotContainer;
    }

    // Configuration
    ///////////////////////////////////////////
    private SubsystemsConfig               subsystemsConfig;

    private AllianceLandmarksConfig        allianceLandmarksConfig;

    // Subsystems
    ///////////////////////////////////////////

    private AlgaeIntakeSubsystem           algaeIntakeSubsystem;

    private ClimberSubsystem               climberSubsystem;

    private CoralIntakeSubsystem           coralIntakeSubsystem;

    private DriveBaseSubsystem             driveBaseSubsystem;

    private ElevatorSubsystem              elevatorSubsystem;

    private ShoulderSubsystem              shoulderSubsystem;

    // Controllers, Commands, and Triggers
    ///////////////////////////////////////////

    private TriggerBindings                triggerBindings;

    private CommandFactory                 commandFactory;

    // Misc
    ///////////////////////////////////////////

    private Alliance                       currentAlliance;

    private final Logger                   log = Logger.getInstance(this.getClass());

    // A chooser for autonomous commands
    private final SendableChooser<Command> chooser;

    private RobotContainer() {
        // Load configuration
        try {
            subsystemsConfig        = ConfigurationLoader.load("subsystems.json", SubsystemsConfig.class);
            allianceLandmarksConfig = ConfigurationLoader.load("alliancelandmarks.json", AllianceLandmarksConfig.class);
        } catch (ConfigurationException e) {
            log.error("Failed to load configuration: " + e.getMessage());
            e.printStackTrace();
        }

        // Initialize the subsystems
        algaeIntakeSubsystem = new AlgaeIntakeSubsystem(subsystemsConfig);
        climberSubsystem     = new ClimberSubsystem(subsystemsConfig);
        coralIntakeSubsystem = new CoralIntakeSubsystem(subsystemsConfig);
        driveBaseSubsystem   = new DriveBaseSubsystem(subsystemsConfig);
        elevatorSubsystem    = new ElevatorSubsystem(subsystemsConfig);
        shoulderSubsystem    = new ShoulderSubsystem(subsystemsConfig);

        // Initialize the controllers and commands
        commandFactory       = new CommandFactory(algaeIntakeSubsystem, climberSubsystem, coralIntakeSubsystem,
                driveBaseSubsystem, elevatorSubsystem, shoulderSubsystem);
        triggerBindings      = new TriggerBindings(allianceLandmarksConfig.getAllianceLandmarkConfig(currentAlliance),
                commandFactory, driveBaseSubsystem);
        triggerBindings.init();

        // SmartDashboard Buttons
        log.dashboard("AutonomousCommand", new AutonomousCommand(driveBaseSubsystem));

        // Register named commands to PathPlanner
        // NamedCommands.registerCommand("ElevatorGoToCommand",
        // elevatorSubsystem.goToCommand(50.0));

        // TODO: Weird way of resolving a circular dependency, maybe Brandon has a
        // better idea
        // TODO: I think a factory will make sense here:
        // https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#static-command-factories
        // elevatorSubsystem.clearToStow = ()->{ return
        // shoulderSubsystem.isStowed(); };
        // shoulderSubsystem.clearToSpin = ()->{ return elevatorSubsystem.isClear();
        // };

        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands

        // Build an auto chooser. This will use Commands.none() as the default option.
        chooser = AutoBuilder.buildAutoChooser();

        log.dashboard("Auto Chooser", chooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // The selected command will be run in autonomous
        return chooser.getSelected();
    }

    public void resetPose(Pose2d pose2d) {
        driveBaseSubsystem.resetPose(pose2d);
    }

    public void opmodeInit(Alliance alliance) {
        currentAlliance = alliance;
        var config = allianceLandmarksConfig.getAllianceLandmarkConfig(alliance);
        triggerBindings.init(config);
    }

    public void configureTestButtonBindings() {
        // TODO: what do we want here?
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /*
         * // Create some buttons log.debug("configureButtonBindings"); // TODO: assign
         * mapping properly gameController.onButtonHold(GameControllerButton.B,
         * shoulderSubsystem.shoulderCommand(-90.0));
         * gameController.onButtonHold(GameControllerButton.A,
         * shoulderSubsystem.shoulderCommand(90.0));
         * gameController.onButtonHold(GameControllerButton.Start,
         * shoulderSubsystem.generateSysIdCommand(2.0, 5.0, 5.0));
         * gameController.onButtonHold(GameControllerButton.X,
         * shoulderSubsystem.shoulderConstant(1.0));
         * gameController.onButtonHold(GameControllerButton.Y,
         * shoulderSubsystem.shoulderConstant(-1.0));
         */
        /*
         * Button Board assignments
         */
        // gameController.onButtonPress (GameControllerButton.Start) RESET GYRO ROTATION
        /*
         * buttonBoardController.onButtonPress(ButtonBoardButton.Travel, new
         * TravelCommand(elevatorSubsystem, shoulderSubsystem));
         * buttonBoardController.onButtonPress(ButtonBoardButton.Lock, new
         * StopCommand(driveBaseSubsystem));
         * buttonBoardController.onButtonHold(ButtonBoardButton.Eject, new
         * TestLoggerCommand("Place Button Pressed"));
         */

        // TODO: wire this as a SelectCommand to fire the appropriate command when state
        // changes; e.g.,
        // new SelectCommand<>(Map.ofEntries(
        // Map.entry(true,
        // new CollectCoralCommand(driveBaseSubsystem, coralIntakeSubsystem,
        // elevatorSubsystem,
        // shoulderSubsystem, getAlgaePose(), getAlgaeLevel())),
        // Map.entry(false, new CollectCoralCommand(driveBaseSubsystem,
        // algaeIntakeSubsystem, elevatorSubsystem,
        // shoulderSubsystem, getAlgaePose(), getAlgaeLevel()))),
        // () -> {
        // return buttonBoardController.isPressed(ButtonBoardButton.Switch);
        // });

        // new
        // Trigger(gameController.button(1)).whileTrue(driveBaseSubsystem.getAngleMotorTestCommand());
        // new Trigger(gameController.button(6)).whileTrue(
        // driveBaseSubsystem.moveAtAngle(() -> gameController.getRawAxis(1) *
        // landmarks.joystickInversion,
        // () -> gameController.getRawAxis(0) * landmarks.joystickInversion, new
        // Rotation2d(0.0)));
        // new Trigger(gameController.button(5))
        // .whileTrue(driveBaseSubsystem.moveTo(new Pose2d(15.0, 6.0, new
        // Rotation2d(Math.PI))));
        // new
        // Trigger(gameController.button(2)).whileTrue(driveBaseSubsystem.moveFacing(
        // () -> gameController.getRawAxis(1) * landmarks.joystickInversion,
        // () -> gameController.getRawAxis(0) * landmarks.joystickInversion, new
        // Translation2d(15.0, 6.0)));
        // new
        // Trigger(gameController.button(3)).whileTrue(driveBaseSubsystem.getDriveMotorTestCommand());

        // new Trigger( gameController.button( 2 ) ).whileTrue( new
        // DriveAngleSetCommand(new Rotation2d( 0.0 )));
        // new
        // Trigger(gameController.button(8)).whileTrue(manipulatorSubsystem.coralIntakeCommand(false));
        // new
        // Trigger(gameController.button(7)).whileTrue(manipulatorSubsystem.coralIntakeCommand(true));
        // While on the left side of the field and need to collect coral from station go
        // to left station
        //
        // Collect coral from coral stations
        // new Trigger(gameController.button(7) )
        // .whileTrue( new CollectCoralCommand( driveBaseSubsystem,
        // coralIntakeSubsystem, elevatorSubsystem, shoulderSubsystem,
        // ()->getCoralStationFacePose(), ()->getCoralStationPose(), 0.5 ) );
        //
        // Place coral on reef when coral switch set and place button pressed
        // new Trigger(gameController.button(7) )
        // .whileTrue( new PlaceCoralCommand( driveBaseSubsystem,
        // coralIntakeSubsystem, elevatorSubsystem, shoulderSubsystem,
        // ()->getReefFacePose( ), ()->getCoralPose(), ()->getCoralLevel() ) );
    }

}
