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
import frc.robot.commands.ClimbDownCommand;
import frc.robot.commands.ClimbUpCommand;
import frc.robot.commands.CollectCoralCommand;
import frc.robot.commands.PlaceCoralCommand;
import frc.robot.commands.PlaceProcessorCommand;
import frc.robot.commands.TestLoggerCommand;
import frc.robot.commands.TravelCommand;
import frc.robot.commands.drivebase.StopCommand;
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

        if (buttonBoardController.isPressed(ButtonBoardButton.L1)) {
            level = 0.25;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.L2)) {
            level = 0.5;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.L3)) {
            level = 0.75;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.L4)) {
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

        if (buttonBoardController.isPressed(ButtonBoardButton.A)
            || buttonBoardController.isPressed(ButtonBoardButton.B)
            || buttonBoardController.isPressed(ButtonBoardButton.E)
            || buttonBoardController.isPressed(ButtonBoardButton.F)
            || buttonBoardController.isPressed(ButtonBoardButton.I)
            || buttonBoardController.isPressed(ButtonBoardButton.J)) {
                level = 0.75;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.C)
            || buttonBoardController.isPressed(ButtonBoardButton.D)
            || buttonBoardController.isPressed(ButtonBoardButton.G)
            || buttonBoardController.isPressed(ButtonBoardButton.H)
            || buttonBoardController.isPressed(ButtonBoardButton.K)
            || buttonBoardController.isPressed(ButtonBoardButton.L)) {
                level = 0.5;
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

        if (buttonBoardController.isPressed(ButtonBoardButton.A) 
            || buttonBoardController.isPressed(ButtonBoardButton.B)) {
                facePose = landmarks.reefFaceAB;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.C) 
            || buttonBoardController.isPressed(ButtonBoardButton.D)) {
                facePose = landmarks.reefFaceCD;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.E) 
            || buttonBoardController.isPressed(ButtonBoardButton.F)) {
                facePose = landmarks.reefFaceEF;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.G) 
            || buttonBoardController.isPressed(ButtonBoardButton.H)) {
                facePose = landmarks.reefFaceGH;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.I) 
            || buttonBoardController.isPressed(ButtonBoardButton.J)) {
                facePose = landmarks.reefFaceIJ;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.K) 
            || buttonBoardController.isPressed(ButtonBoardButton.L)) {
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

        if (buttonBoardController.isPressed(ButtonBoardButton.A)) {
            facePose = landmarks.reefZoneA;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.B)) {
            facePose = landmarks.reefZoneB;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.C)) {
            facePose = landmarks.reefZoneC;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.D)) {
            facePose = landmarks.reefZoneD;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.E)) {
            facePose = landmarks.reefZoneE;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.F)) {
            facePose = landmarks.reefZoneF;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.G)) {
            facePose = landmarks.reefZoneG;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.H)) {
            facePose = landmarks.reefZoneH;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.I)) {
            facePose = landmarks.reefZoneI;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.J)) {
            facePose = landmarks.reefZoneJ;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.K)) {
            facePose = landmarks.reefZoneK;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.L)) {
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

        if (buttonBoardController.isPressed(ButtonBoardButton.A) 
            || buttonBoardController.isPressed(ButtonBoardButton.B)) {
                facePose = landmarks.reefZoneAB;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.C) 
            || buttonBoardController.isPressed(ButtonBoardButton.D)) {
                facePose = landmarks.reefZoneCD;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.E) 
            || buttonBoardController.isPressed(ButtonBoardButton.F)) {
                facePose = landmarks.reefZoneEF;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.G) 
            || buttonBoardController.isPressed(ButtonBoardButton.H)) {
                facePose = landmarks.reefZoneGH;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.I) 
            || buttonBoardController.isPressed(ButtonBoardButton.J)) {
                facePose = landmarks.reefZoneIJ;
        } else if (buttonBoardController.isPressed(ButtonBoardButton.K) 
            || buttonBoardController.isPressed(ButtonBoardButton.L)) {
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

        buttonBoardController.onButtonPress(ButtonBoardButton.Travel,       new TravelCommand(elevatorSubsystem, shoulderSubsystem));
        buttonBoardController.onButtonPress(ButtonBoardButton.Lock,         new StopCommand(driveBaseSubsystem));
        buttonBoardController.onButtonHold (ButtonBoardButton.Place,        new TestLoggerCommand("Place Button Pressed"));
        buttonBoardController.onButtonHold (ButtonBoardButton.Switch,       new TestLoggerCommand("Coral Selected"));
        buttonBoardController.onButtonHold (ButtonBoardButton.ClimbUp,      new ClimbUpCommand(climberSubsystem));
        buttonBoardController.onButtonHold (ButtonBoardButton.ClimbDown,    new ClimbDownCommand(climberSubsystem));
        buttonBoardController.onButtonHold (ButtonBoardButton.L1,           new PlaceCoralCommand(driveBaseSubsystem, coralIntakeSubsystem, elevatorSubsystem, shoulderSubsystem, getReefFacePose(), getCoralPose(), getCoralLevel()));
        buttonBoardController.onButtonHold (ButtonBoardButton.L2,           new PlaceCoralCommand(driveBaseSubsystem, coralIntakeSubsystem, elevatorSubsystem, shoulderSubsystem, getReefFacePose(), getCoralPose(), getCoralLevel()));
        buttonBoardController.onButtonHold (ButtonBoardButton.L3,           new PlaceCoralCommand(driveBaseSubsystem, coralIntakeSubsystem, elevatorSubsystem, shoulderSubsystem, getReefFacePose(), getCoralPose(), getCoralLevel()));
        buttonBoardController.onButtonHold (ButtonBoardButton.L4,           new PlaceCoralCommand(driveBaseSubsystem, coralIntakeSubsystem, elevatorSubsystem, shoulderSubsystem, getReefFacePose(), getCoralPose(), getCoralLevel()));
        buttonBoardController.onButtonHold (ButtonBoardButton.Net,          new TestLoggerCommand("Net Button Pressed"));
        buttonBoardController.onButtonHold (ButtonBoardButton.Processor,    new PlaceProcessorCommand(driveBaseSubsystem, algaeIntakeSubsystem, elevatorSubsystem, shoulderSubsystem, landmarks.processorFace, landmarks.processor));
        buttonBoardController.onButtonHold (ButtonBoardButton.CoralStation, new CollectCoralCommand(driveBaseSubsystem, coralIntakeSubsystem, elevatorSubsystem, shoulderSubsystem, getCoralStationFacePose(), getCoralStationPose()));

        // new Trigger(gameController.button(1)).whileTrue(driveBaseSubsystem.getAngleMotorTestCommand());
        // new Trigger(gameController.button(6)).whileTrue(
        // driveBaseSubsystem.moveAtAngle(() -> gameController.getRawAxis(1) *
        // landmarks.joystickInversion,
        // () -> gameController.getRawAxis(0) * landmarks.joystickInversion, new
        // Rotation2d(0.0)));
        // new Trigger(gameController.button(5))
        // .whileTrue(driveBaseSubsystem.moveTo(new Pose2d(15.0, 6.0, new
        // Rotation2d(Math.PI))));
        // new Trigger(gameController.button(2)).whileTrue(driveBaseSubsystem.moveFacing(
        // () -> gameController.getRawAxis(1) * landmarks.joystickInversion,
        // () -> gameController.getRawAxis(0) * landmarks.joystickInversion, new
        // Translation2d(15.0, 6.0)));
        // new Trigger(gameController.button(3)).whileTrue(driveBaseSubsystem.getDriveMotorTestCommand());

        // new Trigger( gameController.button( 2 ) ).whileTrue( new
        // DriveAngleSetCommand(new Rotation2d( 0.0 )));
        // new Trigger(gameController.button(8)).whileTrue(manipulatorSubsystem.coralIntakeCommand(false));
        // new Trigger(gameController.button(7)).whileTrue(manipulatorSubsystem.coralIntakeCommand(true));
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
