package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutonomousCommand;
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

    // The robot's subsystems
    public final ClimberSubsystem          climberSubsystem     = new ClimberSubsystem();

    public final ElevatorSubsystem         elevatorSubsystem    = new ElevatorSubsystem();

    public final CoralIntakeSubsystem      coralIntakeSubsystem = new CoralIntakeSubsystem();

    public final AlgaeIntakeSubsystem      algaeIntakeSubsystem = new AlgaeIntakeSubsystem();

    public final ShoulderSubsystem         shoulderSubsystem    = new ShoulderSubsystem();

    public final DriveBaseSubsystem        driveBaseSubsystem   = new DriveBaseSubsystem();

    public final AllianceLandmarks         landmarks            = new AllianceLandmarks();

    // Joysticks
    public final CommandJoystick           driverController     = new CommandJoystick(0);

    private Alliance                       currentAlliance;

    // A chooser for autonomous commands
    private final SendableChooser<Command> chooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        // Smartdashboard Subsystems
        SmartDashboard.putData(driveBaseSubsystem);

        // SmartDashboard Buttons
        SmartDashboard.putData("AutonomousCommand", new AutonomousCommand(driveBaseSubsystem));

        // Register named commands to PathPlanner
        NamedCommands.registerCommand("ElevatorGoToCommand", elevatorSubsystem.goToCommand(50.0));

        // TODO: Weird way of resolving a circular dependency, maybe Brandon has a
        // better idea
        // m_elevatorSubsystem.clearToStow = ()->{ return
        // m_shoulderSubsystem.isStowed(); };
        // m_shoulderSubsystem.clearToSpin = ()->{ return m_elevatorSubsystem.isClear();
        // };

        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // m_driveBaseSubsystem.setDefaultCommand(
        // m_driveBaseSubsystem.moveManual(() -> m_driverController.getRawAxis(1) *
        // m_landmarks.joystickInversion,
        // () -> m_driverController.getRawAxis(0) * m_landmarks.joystickInversion,
        // () -> m_driverController.getRawAxis(4)));
        // m_driveBaseSubsystem.setDefaultCommand( m_driveBaseSubsystem.moveAtAngle(()
        // -> m_driverController.getRawAxis(1) * m_landmarks.joystickInversion, () ->
        // m_driverController.getRawAxis(0) * m_landmarks.joystickInversion, new
        // Rotation2d(Math.PI)));

        // Build an auto chooser. This will use Commands.none() as the default option.
        chooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", chooser);
    }

    public void configureTestButtonBindings() {
        // new Trigger( m_driverController.button( 1 ) ).whileTrue( new
        // TestLoggerCommand() );
        // new Trigger( m_driverController.button( 1 ) ).whileTrue(
        // m_driveBaseSubsystem.getAngleMotorTestCommand() );
        // new Trigger( m_driverController.button( 2 ) ).whileTrue(
        // m_driveBaseSubsystem.getDriveMotorTestCommand() );
    }

    public void opmodeInit(Alliance new_alliance) {
        currentAlliance = new_alliance;
        landmarks.newAlliance(currentAlliance);
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

    /**
     * Use this to allow commands to get the elevator height selected for coral from
     * operator.
     *
     * @return the height to place coral at
     */
    private double getCoralLevel() {
        double level = 0.0;

        if (driverController.button(1).getAsBoolean()) {
            level = 0.25;
        } else if (driverController.button(2).getAsBoolean()) {
            level = 0.5;
        } else if (driverController.button(3).getAsBoolean()) {
            level = 0.75;
        } else if (driverController.button(4).getAsBoolean()) {
            level = 1.00;
        }
        return level;
    }

    /**
     * Use this to allow commands to get the elevator height selected for algae from
     * operator.
     *
     * @return the height to pick algae from
     */
    private double getAlgaeLevel() {
        double level = 0.0;

        if (driverController.button(1).getAsBoolean()) {
            level = 0.25;
        } else if (driverController.button(2).getAsBoolean()) {
            level = 0.5;
        } else if (driverController.button(3).getAsBoolean()) {
            level = 0.75;
        } else if (driverController.button(3).getAsBoolean()) {
            level = 1.00;
        }
        return level;
    }

    /**
     * Use this to allow commands to get the reef face to line up with from
     * operator.
     *
     * @return the reef face to line up with
     */
    private Pose2d getReefFacePose() {
        Pose2d facePose = new Pose2d();

        if (driverController.button(1).getAsBoolean() || driverController.button(2).getAsBoolean()) {
            facePose = landmarks.reefFaceAB;
        } else if (driverController.button(3).getAsBoolean() || driverController.button(4).getAsBoolean()) {
            facePose = landmarks.reefFaceCD;
        } else if (driverController.button(5).getAsBoolean() || driverController.button(6).getAsBoolean()) {
            facePose = landmarks.reefFaceEF;
        } else if (driverController.button(7).getAsBoolean() || driverController.button(8).getAsBoolean()) {
            facePose = landmarks.reefFaceGH;
        } else if (driverController.button(9).getAsBoolean() || driverController.button(10).getAsBoolean()) {
            facePose = landmarks.reefFaceIJ;
        } else if (driverController.button(11).getAsBoolean() || driverController.button(12).getAsBoolean()) {
            facePose = landmarks.reefFaceKL;
        }
        return facePose;
    }

    /**
     * Use this to allow commands to get the reef "stalk" pose from operator.
     *
     * @return the reef "stalk" to line up with
     */
    private Pose2d getCoralPose() {
        Pose2d facePose = new Pose2d();

        if (driverController.button(1).getAsBoolean()) {
            facePose = landmarks.reefZoneA;
        } else if (driverController.button(2).getAsBoolean()) {
            facePose = landmarks.reefZoneB;
        } else if (driverController.button(3).getAsBoolean()) {
            facePose = landmarks.reefZoneC;
        } else if (driverController.button(4).getAsBoolean()) {
            facePose = landmarks.reefZoneD;
        } else if (driverController.button(5).getAsBoolean()) {
            facePose = landmarks.reefZoneE;
        } else if (driverController.button(6).getAsBoolean()) {
            facePose = landmarks.reefZoneF;
        } else if (driverController.button(7).getAsBoolean()) {
            facePose = landmarks.reefZoneG;
        } else if (driverController.button(8).getAsBoolean()) {
            facePose = landmarks.reefZoneH;
        } else if (driverController.button(9).getAsBoolean()) {
            facePose = landmarks.reefZoneI;
        } else if (driverController.button(10).getAsBoolean()) {
            facePose = landmarks.reefZoneJ;
        } else if (driverController.button(11).getAsBoolean()) {
            facePose = landmarks.reefZoneK;
        } else if (driverController.button(12).getAsBoolean()) {
            facePose = landmarks.reefZoneL;
        }
        return facePose;
    }

    /**
     * Use this to allow commands to get algae pose from operator.
     *
     * @return the height currently selected
     */
    private Pose2d getAlgaePose() {
        Pose2d facePose = new Pose2d();

        if (driverController.button(1).getAsBoolean() || driverController.button(2).getAsBoolean()) {
            facePose = landmarks.reefZoneAB;
        } else if (driverController.button(3).getAsBoolean() || driverController.button(4).getAsBoolean()) {
            facePose = landmarks.reefZoneCD;
        } else if (driverController.button(5).getAsBoolean() || driverController.button(6).getAsBoolean()) {
            facePose = landmarks.reefZoneEF;
        } else if (driverController.button(7).getAsBoolean() || driverController.button(8).getAsBoolean()) {
            facePose = landmarks.reefZoneGH;
        } else if (driverController.button(9).getAsBoolean() || driverController.button(10).getAsBoolean()) {
            facePose = landmarks.reefZoneIJ;
        } else if (driverController.button(11).getAsBoolean() || driverController.button(12).getAsBoolean()) {
            facePose = landmarks.reefZoneKL;
        }
        return facePose;
    }

    /**
     * Use this to allow commands to get algae pose from operator.
     *
     * @return the height currently selected
     */
    private Pose2d getCoralStationFacePose() {
        Pose2d facePose = new Pose2d();

        if (driveBaseSubsystem.getPose().getX() <= 4.0386) {
            facePose = landmarks.coralStationLeftFace;
        } else {
            facePose = landmarks.coralStationRightFace;
        }
        return facePose;
    }

    /**
     * Use this to allow commands to get algae pose from operator.
     *
     * @return the height currently selected
     */
    private Pose2d getCoralStationPose() {
        Pose2d facePose = new Pose2d();

        if (driveBaseSubsystem.getPose().getX() <= 4.0386) {
            facePose = landmarks.coralStationLeft;
        } else {
            facePose = landmarks.coralStationRight;
        }
        return facePose;
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Create some buttons

        System.out.println("configureButtonBindings");
        // new
        // Trigger(m_driverController.button(1)).whileTrue(m_driveBaseSubsystem.getAngleMotorTestCommand());
        // new Trigger(m_driverController.button(6)).whileTrue(
        // m_driveBaseSubsystem.moveAtAngle(() -> m_driverController.getRawAxis(1) *
        // m_landmarks.joystickInversion,
        // () -> m_driverController.getRawAxis(0) * m_landmarks.joystickInversion, new
        // Rotation2d(0.0)));
        // new Trigger(m_driverController.button(5))
        // .whileTrue(m_driveBaseSubsystem.moveTo(new Pose2d(15.0, 6.0, new
        // Rotation2d(Math.PI))));
        // new
        // Trigger(m_driverController.button(2)).whileTrue(m_driveBaseSubsystem.moveFacing(
        // () -> m_driverController.getRawAxis(1) * m_landmarks.joystickInversion,
        // () -> m_driverController.getRawAxis(0) * m_landmarks.joystickInversion, new
        // Translation2d(15.0, 6.0)));
        // new
        // Trigger(m_driverController.button(3)).whileTrue(m_driveBaseSubsystem.getDriveMotorTestCommand());
        new Trigger(driverController.button(8)).whileTrue(shoulderSubsystem.shoulderCommand(-90.0));
        new Trigger(driverController.button(7)).whileTrue(shoulderSubsystem.shoulderCommand(90.0));
        new Trigger(driverController.button(1)).whileTrue(shoulderSubsystem.generateSysIdCommand(2.0, 10.0, 3.0));
        // new Trigger( m_driverController.button( 2 ) ).whileTrue( new
        // DriveAngleSetCommand(new Rotation2d( 0.0 )));
        // new
        // Trigger(m_driverController.button(8)).whileTrue(m_manipulatorSubsystem.coralIntakeCommand(false));
        // new
        // Trigger(m_driverController.button(7)).whileTrue(m_manipulatorSubsystem.coralIntakeCommand(true));
        // While on the left side of the field and need to collect coral from station go
        // to left station
        //
        // Collect coral from coral stations
        // new Trigger(m_driverController.button(7) )
        // .whileTrue( new CollectCoralCommand( m_driveBaseSubsystem,
        // m_coralIntakeSubsystem, m_elevatorSubsystem, m_shoulderSubsystem,
        // ()->getCoralStationFacePose(), ()->getCoralStationPose(), 0.5 ) );
        //
        // Place coral on reef when coral switch set and place button pressed
        // new Trigger(m_driverController.button(7) )
        // .whileTrue( new PlaceCoralCommand( m_driveBaseSubsystem,
        // m_coralIntakeSubsystem, m_elevatorSubsystem, m_shoulderSubsystem,
        // ()->getReefFacePose( ), ()->getCoralPose(), ()->getCoralLevel() ) );
    }

}
