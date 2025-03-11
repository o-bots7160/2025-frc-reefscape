package frc.robot.helpers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveBaseSubsystem;

@Logged
public class Pathplanner {

    private DriveBaseSubsystem       driveBaseSubsystem;

    private Logger                   log                   = Logger.getInstance(this.getClass());

    private boolean                  autoBuilderConfigured = false;

    private SendableChooser<Command> autoBuilderChooser;

    public Pathplanner(DriveBaseSubsystem driveBaseSubsystem) {
        this.driveBaseSubsystem = driveBaseSubsystem;
        configureAutoBuilder();
    }

    public SendableChooser<Command> getAutonomousChooser() {
        if (driveBaseSubsystem.checkDisabled() || !autoBuilderConfigured) {
            // if we're disabled or not configured, there's nothing to choose
            return new SendableChooser<>();
        }

        return autoBuilderChooser;
    }

    /**
     * Configures AutoBuilder
     */
    private void configureAutoBuilder() {
        try {
            var robotConfig = RobotConfig.fromGUISettings();

            // TODO: The docs say it should be done in the drivebase subsystem: https://pathplanner.dev/pplib-build-an-auto.html
            AutoBuilder.configure(
                    // Robot pose supplier
                    driveBaseSubsystem::getPose,
                    // Method to reset odometry (will be called if your auto has a starting pose)
                    driveBaseSubsystem::resetPose,
                    // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    driveBaseSubsystem::getRobotRelativeSpeeds,
                    // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
                    // optionally outputs individual module feedforwards
                    (speeds, feedforwards) -> driveBaseSubsystem.swerveDrive.drive(speeds,
                            driveBaseSubsystem.swerveDrive.kinematics.toSwerveModuleStates(speeds),
                            feedforwards.linearForces()),
                    // PPHolonomicController is the built in path following controller for holonomic
                    // drive trains
                    new PPHolonomicDriveController(
                            // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0),
                            // Rotation PID constants
                            new PIDConstants(5.0, 0.0, 0.0)),
                    // The robot configuration
                    robotConfig,
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    // Reference to this subsystem to set requirements
                    driveBaseSubsystem);

            autoBuilderConfigured = true;
            autoBuilderChooser    = AutoBuilder.buildAutoChooser();

            log.dashboard("Auto Chooser", autoBuilderChooser);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
