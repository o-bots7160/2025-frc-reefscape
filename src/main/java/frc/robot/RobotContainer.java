package frc.robot;

import javax.naming.ConfigurationException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CommandFactory;
import frc.robot.config.AllianceLandmarkConfig;
import frc.robot.config.AllianceLandmarksConfig;
import frc.robot.config.ConfigurationLoader;
import frc.robot.config.SubsystemsConfig;
import frc.robot.helpers.CommandRegister;
import frc.robot.helpers.Logger;
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
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // The selected command will be run in autonomous
        var chooser = commandFactory.getAutonomousChooser();

        return chooser.getSelected();
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
        commandRegister = new CommandRegister(allianceConfig, commandFactory, triggerBindings);
    }

}
