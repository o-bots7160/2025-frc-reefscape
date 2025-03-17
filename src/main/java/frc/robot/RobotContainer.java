package frc.robot;

import javax.naming.ConfigurationException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.drivebase.MoveToCommand;
import frc.robot.config.AllianceLandmarkConfig;
import frc.robot.config.AllianceLandmarksConfig;
import frc.robot.config.ConfigurationLoader;
import frc.robot.config.SubsystemsConfig;
import frc.robot.helpers.CommandRegister;
import frc.robot.helpers.Logger;
import frc.robot.helpers.Pathplanner;
import frc.robot.helpers.TriggerBindings;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
// @Logged
public class RobotContainer {

    private static RobotContainer robotContainer = new RobotContainer();

    public static RobotContainer getInstance() {
        return robotContainer;
    }

    // Configuration
    ///////////////////////////////////////////
    private SubsystemsConfig        subsystemsConfig;

    private AllianceLandmarksConfig allianceLandmarksConfig;

    // Subsystems
    ///////////////////////////////////////////

    private AlgaeIntakeSubsystem    algaeIntakeSubsystem;

    private ClimberSubsystem        climberSubsystem;

    private CoralIntakeSubsystem    coralIntakeSubsystem;

    private DriveBaseSubsystem      driveBaseSubsystem;

    private ElevatorSubsystem       elevatorSubsystem;

    private ShoulderSubsystem       shoulderSubsystem;

    // Controllers, Commands, and Triggers
    ///////////////////////////////////////////

    private TriggerBindings         triggerBindings;

    private CommandRegister         commandRegister;

    private Pathplanner             pathplanner;

    private SendableChooser<Command> autonChooser;

    private CommandFactory          commandFactory;

    // Misc
    ///////////////////////////////////////////

    private Alliance                currentAlliance = Alliance.Blue;

    private final Logger            log             = Logger.getInstance(this.getClass());

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

        configureCommandsAndTriggers();

        // CREATE AUTON OPTIONS
        // TODO: this is not the place for it, but it will have to work
        var                      redAllianceLandmarks  = allianceLandmarksConfig.getRedAlliance();
        var                      blueAllianceLandmarks = allianceLandmarksConfig.getBlueAlliance();
        autonChooser          = new SendableChooser<Command>();
        autonChooser.addOption("Red Left Side Move", new SequentialCommandGroup(
                new MoveToCommand(driveBaseSubsystem, redAllianceLandmarks.reefFaceIJ),
                commandFactory.createPlaceCoralCommand(null, "4", null,
                        () -> redAllianceLandmarks.coralLevel4, () -> redAllianceLandmarks.coralLevel4Rotation)));
        autonChooser.addOption("Red Middle Move", new SequentialCommandGroup(
                new MoveToCommand(driveBaseSubsystem, redAllianceLandmarks.reefFaceGH),
                commandFactory.createPlaceCoralCommand(null, "4", null,
                        () -> redAllianceLandmarks.coralLevel4, () -> redAllianceLandmarks.coralLevel4Rotation)));
        autonChooser.addOption("Red Right Side Move", new SequentialCommandGroup(
                new MoveToCommand(driveBaseSubsystem, redAllianceLandmarks.reefFaceEF),
                commandFactory.createPlaceCoralCommand(null, "4", null,
                        () -> redAllianceLandmarks.coralLevel4, () -> redAllianceLandmarks.coralLevel4Rotation)));

        autonChooser.addOption("Blue Left Side Move", new SequentialCommandGroup(
                new MoveToCommand(driveBaseSubsystem, blueAllianceLandmarks.reefFaceIJ),
                commandFactory.createPlaceCoralCommand(null, "4", null,
                        () -> blueAllianceLandmarks.coralLevel4, () -> blueAllianceLandmarks.coralLevel4Rotation)));
        autonChooser.addOption("Blue Middle Move", new SequentialCommandGroup(
                new MoveToCommand(driveBaseSubsystem, blueAllianceLandmarks.reefFaceGH),
                commandFactory.createPlaceCoralCommand(null, "4", null,
                        () -> blueAllianceLandmarks.coralLevel4, () -> blueAllianceLandmarks.coralLevel4Rotation)));
        autonChooser.addOption("Blue Right Side Move", new SequentialCommandGroup(
                new MoveToCommand(driveBaseSubsystem, blueAllianceLandmarks.reefFaceEF),
                commandFactory.createPlaceCoralCommand(null, "4", null,
                        () -> blueAllianceLandmarks.coralLevel4, () -> blueAllianceLandmarks.coralLevel4Rotation)));

        log.dashboard("Auton Chooser", autonChooser);
    }

    public void enable(boolean new_state) {
        driveBaseSubsystem.enable(new_state);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // The selected command will be run in autonomous
        // var chooser = pathplanner.getAutonomousChooser();

        var selectedCommand =  autonChooser.getSelected();
        return selectedCommand;
    }

    public void resetPose(Pose2d pose2d) {
        driveBaseSubsystem.resetPose(pose2d);
    }

    public void opmodeInit(Alliance alliance) {
        if (alliance != currentAlliance) {
            log.info("Alliance changed from " + currentAlliance + " to " + alliance);
            currentAlliance = alliance;
            configureCommandsAndTriggers();
        }
        log.dashboard("Current Alliance", alliance.name());
    }

    public void configureTestButtonBindings() {
        // TODO: what do we want here?
    }

    private void configureCommandsAndTriggers() {

        // Initialize the controllers and commands
        AllianceLandmarkConfig allianceConfig = allianceLandmarksConfig.getAllianceLandmarkConfig(currentAlliance);
        commandFactory  = new CommandFactory(
                // Subsystems
                algaeIntakeSubsystem, climberSubsystem, coralIntakeSubsystem, driveBaseSubsystem, elevatorSubsystem, shoulderSubsystem,
                // Config
                allianceConfig);
        triggerBindings = new TriggerBindings(allianceConfig, commandFactory, driveBaseSubsystem);
        commandRegister = new CommandRegister(allianceConfig, commandFactory);

        // TODO: Throws an exception if done more than once
        // pathplanner     = new Pathplanner(driveBaseSubsystem, commandFactory, allianceConfig);
    }

}
