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
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.TestLoggerCommand;
import frc.robot.devices.ButtonBoardController;
import frc.robot.devices.ButtonBoardController.ButtonBoardButton;
import frc.robot.devices.GameController;
import frc.robot.devices.GameController.GameControllerButton;
import frc.robot.helpers.Logger;
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
    public final ClimberSubsystem          climberSubsystem      = new ClimberSubsystem();

    public final ElevatorSubsystem         elevatorSubsystem     = new ElevatorSubsystem();

    public final CoralIntakeSubsystem      coralIntakeSubsystem  = new CoralIntakeSubsystem();

    public final AlgaeIntakeSubsystem      algaeIntakeSubsystem  = new AlgaeIntakeSubsystem();

    public final ShoulderSubsystem         shoulderSubsystem     = new ShoulderSubsystem();

    public final DriveBaseSubsystem        driveBaseSubsystem    = new DriveBaseSubsystem();

    public final AllianceLandmarks         landmarks             = new AllianceLandmarks();

    // Joysticks
    public final GameController            gameController        = new GameController(0);

    public final ButtonBoardController     buttonBoardController = new ButtonBoardController(1, 2, 3, 4);

    protected Logger                       log                   = Logger.getInstance(this.getClass());

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
        // TODO: I think a factory will make sense here:
        // https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#static-command-factories
        // elevatorSubsystem.clearToStow = ()->{ return
        // shoulderSubsystem.isStowed(); };
        // shoulderSubsystem.clearToSpin = ()->{ return elevatorSubsystem.isClear();
        // };

        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // driveBaseSubsystem.setDefaultCommand(
        // driveBaseSubsystem.moveManual(() -> gameController.getRawAxis(1) *
        // landmarks.joystickInversion,
        // () -> gameController.getRawAxis(0) * landmarks.joystickInversion,
        // () -> gameController.getRawAxis(4)));
        // driveBaseSubsystem.setDefaultCommand( driveBaseSubsystem.moveAtAngle(()
        // -> gameController.getRawAxis(1) * landmarks.joystickInversion, () ->
        // gameController.getRawAxis(0) * landmarks.joystickInversion, new
        // Rotation2d(Math.PI)));

        // Build an auto chooser. This will use Commands.none() as the default option.
        chooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", chooser);
    }

    public void configureTestButtonBindings() {
        // new Trigger( gameController.button( 1 ) ).whileTrue( new
        // TestLoggerCommand() );
        // new Trigger( gameController.button( 1 ) ).whileTrue(
        // driveBaseSubsystem.getAngleMotorTestCommand() );
        // new Trigger( gameController.button( 2 ) ).whileTrue(
        // driveBaseSubsystem.getDriveMotorTestCommand() );
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

        if (gameController.button(1).getAsBoolean()) {
            level = 0.25;
        } else if (gameController.button(2).getAsBoolean()) {
            level = 0.5;
        } else if (gameController.button(3).getAsBoolean()) {
            level = 0.75;
        } else if (gameController.button(4).getAsBoolean()) {
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

        if (gameController.button(1).getAsBoolean()) {
            level = 0.25;
        } else if (gameController.button(2).getAsBoolean()) {
            level = 0.5;
        } else if (gameController.button(3).getAsBoolean()) {
            level = 0.75;
        } else if (gameController.button(3).getAsBoolean()) {
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

        if (gameController.button(1).getAsBoolean() || gameController.button(2).getAsBoolean()) {
            facePose = landmarks.reefFaceAB;
        } else if (gameController.button(3).getAsBoolean() || gameController.button(4).getAsBoolean()) {
            facePose = landmarks.reefFaceCD;
        } else if (gameController.button(5).getAsBoolean() || gameController.button(6).getAsBoolean()) {
            facePose = landmarks.reefFaceEF;
        } else if (gameController.button(7).getAsBoolean() || gameController.button(8).getAsBoolean()) {
            facePose = landmarks.reefFaceGH;
        } else if (gameController.button(9).getAsBoolean() || gameController.button(10).getAsBoolean()) {
            facePose = landmarks.reefFaceIJ;
        } else if (gameController.button(11).getAsBoolean() || gameController.button(12).getAsBoolean()) {
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

        if (gameController.button(1).getAsBoolean()) {
            facePose = landmarks.reefZoneA;
        } else if (gameController.button(2).getAsBoolean()) {
            facePose = landmarks.reefZoneB;
        } else if (gameController.button(3).getAsBoolean()) {
            facePose = landmarks.reefZoneC;
        } else if (gameController.button(4).getAsBoolean()) {
            facePose = landmarks.reefZoneD;
        } else if (gameController.button(5).getAsBoolean()) {
            facePose = landmarks.reefZoneE;
        } else if (gameController.button(6).getAsBoolean()) {
            facePose = landmarks.reefZoneF;
        } else if (gameController.button(7).getAsBoolean()) {
            facePose = landmarks.reefZoneG;
        } else if (gameController.button(8).getAsBoolean()) {
            facePose = landmarks.reefZoneH;
        } else if (gameController.button(9).getAsBoolean()) {
            facePose = landmarks.reefZoneI;
        } else if (gameController.button(10).getAsBoolean()) {
            facePose = landmarks.reefZoneJ;
        } else if (gameController.button(11).getAsBoolean()) {
            facePose = landmarks.reefZoneK;
        } else if (gameController.button(12).getAsBoolean()) {
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

        if (gameController.button(1).getAsBoolean() || gameController.button(2).getAsBoolean()) {
            facePose = landmarks.reefZoneAB;
        } else if (gameController.button(3).getAsBoolean() || gameController.button(4).getAsBoolean()) {
            facePose = landmarks.reefZoneCD;
        } else if (gameController.button(5).getAsBoolean() || gameController.button(6).getAsBoolean()) {
            facePose = landmarks.reefZoneEF;
        } else if (gameController.button(7).getAsBoolean() || gameController.button(8).getAsBoolean()) {
            facePose = landmarks.reefZoneGH;
        } else if (gameController.button(9).getAsBoolean() || gameController.button(10).getAsBoolean()) {
            facePose = landmarks.reefZoneIJ;
        } else if (gameController.button(11).getAsBoolean() || gameController.button(12).getAsBoolean()) {
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

        log.debug("configureButtonBindings");

        // TODO: assign mapping properly
        gameController.onButtonHold(GameControllerButton.B, shoulderSubsystem.shoulderCommand(-90.0));
        gameController.onButtonHold(GameControllerButton.A, shoulderSubsystem.shoulderCommand(90.0));
        gameController.onButtonHold(GameControllerButton.Start, shoulderSubsystem.generateSysIdCommand(2.0, 5.0, 5.0));
        gameController.onButtonHold(GameControllerButton.X, shoulderSubsystem.shoulderConstant(1.0));
        gameController.onButtonHold(GameControllerButton.Y, shoulderSubsystem.shoulderConstant(-1.0));

        /*
         * Button Board assignments
         */
        // gameController.onButtonPress (GameControllerButton.Start) RESET GYRO ROTATION

        buttonBoardController.onButtonPress(ButtonBoardButton.Travel,       new TestLoggerCommand("Travel Button Pressed"       ));
        buttonBoardController.onButtonPress(ButtonBoardButton.Lock,         new TestLoggerCommand("Lock Button Pressed"         ));
        buttonBoardController.onButtonHold (ButtonBoardButton.Place,        new TestLoggerCommand("Place Button Pressed"        ));
        buttonBoardController.onButtonHold (ButtonBoardButton.A,            new TestLoggerCommand("Reef Position A Selected"    ));
        buttonBoardController.onButtonHold (ButtonBoardButton.B,            new TestLoggerCommand("Reef Position B Selected"    ));
        buttonBoardController.onButtonHold (ButtonBoardButton.C,            new TestLoggerCommand("Reef Position C Selected"    ));
        buttonBoardController.onButtonHold (ButtonBoardButton.D,            new TestLoggerCommand("Reef Position D Selected"    ));
        buttonBoardController.onButtonHold (ButtonBoardButton.E,            new TestLoggerCommand("Reef Position E Selected"    ));
        buttonBoardController.onButtonHold (ButtonBoardButton.F,            new TestLoggerCommand("Reef Position F Selected"    ));
        buttonBoardController.onButtonHold (ButtonBoardButton.G,            new TestLoggerCommand("Reef Position G Selected"    ));
        buttonBoardController.onButtonHold (ButtonBoardButton.H,            new TestLoggerCommand("Reef Position H Selected"    ));
        buttonBoardController.onButtonHold (ButtonBoardButton.I,            new TestLoggerCommand("Reef Position I Selected"    ));
        buttonBoardController.onButtonHold (ButtonBoardButton.J,            new TestLoggerCommand("Reef Position J Selected"    ));
        buttonBoardController.onButtonHold (ButtonBoardButton.K,            new TestLoggerCommand("Reef Position K Selected"    ));
        buttonBoardController.onButtonHold (ButtonBoardButton.L,            new TestLoggerCommand("Reef Position L Selected"    ));
        buttonBoardController.onButtonHold (ButtonBoardButton.L1,           new TestLoggerCommand("Level 1 Button Pressed"      ));
        buttonBoardController.onButtonHold (ButtonBoardButton.L2,           new TestLoggerCommand("Level 2 Button Pressed"      ));
        buttonBoardController.onButtonHold (ButtonBoardButton.L3,           new TestLoggerCommand("Level 3 Button Pressed"      ));
        buttonBoardController.onButtonHold (ButtonBoardButton.L4,           new TestLoggerCommand("Level 4 Button Pressed"      ));
        buttonBoardController.onButtonHold (ButtonBoardButton.Switch,       new TestLoggerCommand("Coral Selected"              ));
        buttonBoardController.onButtonHold (ButtonBoardButton.ClimbUp,      new TestLoggerCommand("Climb Up Button Pressed"     ));
        buttonBoardController.onButtonHold (ButtonBoardButton.ClimbDown,    new TestLoggerCommand("Climb Down Button Pressed"   ));
        buttonBoardController.onButtonHold (ButtonBoardButton.Net,          new TestLoggerCommand("Net Button Pressed"          ));
        buttonBoardController.onButtonHold (ButtonBoardButton.Processor,    new TestLoggerCommand("Processor Button Pressed"    ));
        buttonBoardController.onButtonHold (ButtonBoardButton.CoralStation, new TestLoggerCommand("Coral Station Button Pressed"));

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
