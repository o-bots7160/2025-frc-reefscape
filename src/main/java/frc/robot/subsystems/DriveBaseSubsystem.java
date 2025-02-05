package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.commands.drivebase.MoveAtAngle;
import frc.robot.commands.drivebase.MoveFacingCommand;
import frc.robot.commands.drivebase.MoveManualCommandField;
import frc.robot.commands.drivebase.MoveToCommand;
import frc.robot.commands.drivebase.StopCommand;
import frc.robot.config.AllianceLandmarksConfig;
import frc.robot.config.ConfigurationLoader;
import frc.robot.config.DriveBaseSubsystemConfig;
import frc.robot.helpers.LimelightDevice;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * The drive base subsystem for the robot.
 */
@Logged
public class DriveBaseSubsystem extends SubsystemBase {
    private static double            kDt                    = 0.02;

    LimelightDevice                  primaryLimelight       = new LimelightDevice("limelight");

    boolean                          hasTarget              = true;

    SwerveDrive                      swerveDrive;

    private boolean                  isSimulation           = !RobotBase.isReal();

    private Translation2d            centerOfRotationMeters = new Translation2d();

    private final TrapezoidProfile   xy_profile;

    private Translation2d            xy_speed               = new Translation2d();

    private Translation2d            xy_target              = new Translation2d();

    private TrapezoidProfile.State   xy_goal                = new TrapezoidProfile.State();

    private TrapezoidProfile.State   xy_setpoint            = new TrapezoidProfile.State();

    private double                   xy_last                = 0.0;

    private PIDController            xy_PID                 = new PIDController(6.0, 0.0, 0.0);

    private final TrapezoidProfile   r_profile              = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(30.0, 4.5));                                      // TODO:
    // Maxrotational
    // speed/accel?

    private double                   r_speed                = 0.0;

    private Rotation2d               r_target               = new Rotation2d();

    private TrapezoidProfile.State   r_goal                 = new TrapezoidProfile.State();

    private TrapezoidProfile.State   r_setpoint             = new TrapezoidProfile.State();

    private double                   r_last                 = 0.0;

    private PIDController            r_PID                  = new PIDController(6.0, 0.0, 0.0);

    private SwerveController         swerveController;

    private DriveBaseSubsystemConfig driveBaseSubsystemConfig;

    private AllianceLandmarksConfig  allianceLandmarksConfig;

    /**
     * Constructor
     */
    public DriveBaseSubsystem() {
        try {
            // Load Configuration
            driveBaseSubsystemConfig = ConfigurationLoader.load("drivebasesubsystem.json",
                    DriveBaseSubsystemConfig.class);

            allianceLandmarksConfig  = ConfigurationLoader.load("alliancelandmarks.json",
                    AllianceLandmarksConfig.class);

            configureSwerveDrive();
            configureAutoBuilder();

        } catch (Exception e) {
            e.printStackTrace();
        }

        // Configure Swerve Controller
        //////////////////////////////////////////////////

        xy_PID.setTolerance(0.05, 0.05);
        xy_PID.setIntegratorRange(-0.04, 0.04);
        xy_PID.setSetpoint(0);

        r_PID.setTolerance(0.05, 0.05);
        r_PID.setIntegratorRange(-0.04, 0.04);
        r_PID.setSetpoint(0);

        xy_profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(driveBaseSubsystemConfig.getMaximumSpeedInMeters(), 3.0)); // TODO: Max
                                                                                                            // linear
                                                                                                            // accel?
    }

    /**
     * Called once per timeslice
     *
     * @return void
     */
    @Override
    public void periodic() {
        Pose2d current_pose = swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition();

        if (!isSimulation) {
            limelightPeriodic(current_pose.getRotation().getDegrees());
        }

        SmartDashboard.putNumber("RobotX", current_pose.getX());
        SmartDashboard.putNumber("RobotY", current_pose.getY());
        SmartDashboard.putNumber("RobotRot", current_pose.getRotation().getDegrees());
    }

    /**
     * Called once per timeslice while simulating
     *
     * @return void
     */
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    /**
     * Returns the latest pose of the robot from odometery
     *
     * @return current pose of robot
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Resets the Odometry of the Swerve Drive
     * 
     * @param new_pose
     */
    public void resetPose(Pose2d new_pose) {
        swerveDrive.resetOdometry(new_pose);
    }

    /**
     * Get the velocity of the robot from the Swerve Drive
     * 
     * @return
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Sets the current pose of the robot from odometery (usually at the start of
     * auton)
     *
     * @param new_pose of the robot
     * @return void
     */
    public void setPose(Pose2d new_pose) {
        swerveDrive.swerveDrivePoseEstimator.resetPose(new_pose);
    }

    /**
     * @return a Command for manual control
     */
    public Command stopManual() {
        return new StopCommand(this).raceWith(new WaitCommand(1.0));
    }

    /**
     * @return a Command for manual control
     */
    public Command moveManual(DoubleSupplier new_x, DoubleSupplier new_y, DoubleSupplier new_rot) {
        return new MoveManualCommandField(this, new_x, new_y, new_rot);
    }

    /**
     * @return a Command for manual control of position while facing a Pose2d on the
     *         field
     */
    public Command moveAtAngle(DoubleSupplier new_x, DoubleSupplier new_y, Rotation2d new_rot) {
        return new MoveAtAngle(this, new_x, new_y, new_rot);
    }

    /**
     * @return a Command to go to a Pose2d
     */
    public Command moveTo(Pose2d new_pose) {
        return new MoveToCommand(this, new_pose);
    }

    /**
     * @return a Command for manual control of position while facing a Pose2d on the
     *         field
     */
    public Command moveFacing(DoubleSupplier new_x, DoubleSupplier new_y, Translation2d new_translation) {
        return new MoveFacingCommand(this, new_x, new_y, new_translation);
    }

    /**
     * Return a Command to test the angle motors
     */
    public Command getAngleMotorTestCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive), 3.0, 4.0, 4.0);
    }

    /**
     * Return a Command to test the drive motors
     */
    public Command getDriveMotorTestCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 6.0, false), 3.0, 4.0, 4.0);
    }

    /**
     * Drive the robot using robot-oriented control
     *
     * @param x   the x meters per second to move
     * @param y   the y meters per second to move
     * @param rot the radians per second to move
     * @return void
     */
    public void driveRobot(double x, double y, double r) {
        xy_speed = new Translation2d(x, y);
        r_speed  = r;
        drive(false, false);
    }

    /**
     * Drive the robot using field-oriented control
     *
     * @param x   the x meters per second to move
     * @param y   the y meters per second to move
     * @param rot the radians per second to move
     * @return void
     */
    public void driveField(double x, double y, double r) {
        xy_speed = new Translation2d(x, y);
        r_speed  = r;
        drive(true, false);
    }

    /**
     * Stop the robot by setting chassis speeds to 0.0
     *
     * @return void
     */
    public void stop() {
        hasTarget   = true;
        xy_setpoint = new TrapezoidProfile.State();
        r_setpoint  = new TrapezoidProfile.State();
        driveField(0.0, 0.0, 0.0);
        swerveDrive.lockPose();
    }

    /**
     * Sets the target rotation for the robot (usually at the start of a command)
     *
     * @param new_target for the robot
     * @return void
     */
    public void setTarget(Rotation2d new_target, Rotation2d current_pose) {
        r_target   = new_target;
        r_last     = MathUtil.angleModulus(r_target.getRadians() - current_pose.getRadians());
        r_setpoint = new TrapezoidProfile.State(r_last, r_speed);
        r_PID.reset();
        hasTarget = false;
    }

    /**
     * Drive facing target pose
     *
     * @return void
     */
    public void driveAtAngle(double x, double y) {
        Pose2d current_pose = getPose();

        xy_speed = new Translation2d(x, y);

        if (setRotationSpeedFromTarget(current_pose.getRotation())) {
            hasTarget = true;
        } else {
            hasTarget = false;
        }
        drive(true, false);
    }

    /**
     * Drive facing target pose
     *
     * @return void
     */
    public void driveFacingTarget(double x, double y) {
        Pose2d current_pose = getPose();

        xy_speed = new Translation2d(x, y);
        setTarget(
                new Rotation2d(
                        Math.atan2(xy_target.getY() - current_pose.getY(), xy_target.getX() - current_pose.getX())),
                current_pose.getRotation());
        if (setRotationSpeedFromTarget(current_pose.getRotation())) {
            hasTarget = true;
        } else {
            hasTarget = false;
        }
        drive(true, false);
    }

    /**
     * Sets the target translation for the robot (usually at the start of a command)
     *
     * @param new_target for the robot
     * @return void
     */
    public void setTarget(Translation2d new_target, Translation2d current_pose) {
        xy_target   = new_target;
        xy_last     = Math.hypot(xy_target.getX() - current_pose.getX(), xy_target.getY() - current_pose.getY());
        xy_setpoint = new TrapezoidProfile.State(xy_last, xy_setpoint.velocity);
        xy_PID.reset();
        hasTarget = false;
    }

    /**
     * Sets the target pose for the robot (usually at the start of a command)
     *
     * @param new_target for the robot
     * @return void
     */
    public void setTarget(Pose2d new_target, Pose2d current_pose) {
        setTarget(new_target.getTranslation(), current_pose.getTranslation());
        setTarget(new_target.getRotation(), current_pose.getRotation());
    }

    /**
     * Returns the state of a target being aquired
     * 
     * @return
     */
    public boolean getHasTarget() {
        return hasTarget;
    }

    /**
     * Drive towards target pose
     *
     * @return void
     */
    public void driveToTarget() {
        boolean at_xy, at_r;
        Pose2d  current_pose = getPose();

        at_xy = setXYSpeedsFromTarget(current_pose.getTranslation());
        at_r  = setRotationSpeedFromTarget(current_pose.getRotation());

        if (!at_xy || !at_r) {
            hasTarget = false;
        } else {
            hasTarget = true;
        }
        drive(true, false);
    }

    /**
     * Locks the Swerve Drive pose
     */
    public void lockSwerveDrivePose() {
        swerveDrive.lockPose();
    }

    /**
     * Loads configuration files and configures the Swerve Drive
     */
    private void configureSwerveDrive() {
        try {
            swerveDrive      = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
                    .createSwerveDrive(driveBaseSubsystemConfig.getMaximumSpeedInMeters());
            swerveController = swerveDrive.swerveController;
            swerveController.thetaController.setTolerance(Math.PI / driveBaseSubsystemConfig.thetaControllerTolerance,
                    0.1);
            swerveController.thetaController.setPID(driveBaseSubsystemConfig.thetaControllerPidKp,
                    driveBaseSubsystemConfig.thetaControllerPidKi, driveBaseSubsystemConfig.thetaControllerPidKd);

            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

            swerveDrive.setMotorIdleMode(true);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Configures AutoBuilder
     */
    private void configureAutoBuilder() {
        try {
            var robotConfig = RobotConfig.fromGUISettings();

            AutoBuilder.configure(this::getPose, // Robot pose supplier
                    this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> swerveDrive.drive(speeds, // Method that will drive the robot given ROBOT
                                                                        // RELATIVE ChassisSpeeds. Also optionally
                                                                        // outputs
                                                                        // individual module feedforwards
                            swerveDrive.kinematics.toSwerveModuleStates(speeds), feedforwards.linearForces()),
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
                                                    // for
                                                    // holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                    ), robotConfig, // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    }, this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Gets estimated pose from limelight if available
     *
     * @param degrees angle robot is facing
     * @return void
     */
    private void limelightPeriodic(double degrees) {

        var primaryLightLightPose = primaryLimelight.getPoseEstimate(degrees);
        if (primaryLightLightPose.tagCount != 0) {
            swerveDrive.addVisionMeasurement(primaryLightLightPose.pose, primaryLightLightPose.timestampSeconds);
        }
    }

    /**
     * Issue set speeds to swerve drive
     *
     * @return void
     */
    private void drive(boolean fieldRelative, boolean isOpenLoop) {
        swerveDrive.drive(xy_speed, r_speed, fieldRelative, isOpenLoop, centerOfRotationMeters);
    }

    /**
     * Set X and Y speeds for swerve drive base on distance from target
     *
     * @param currentPose of the robot
     * @return boolean true if within deadband otherwise false
     */
    private boolean setXYSpeedsFromTarget(Translation2d currentPose) {
        Double  x_err    = xy_target.getX() - currentPose.getX();
        Double  y_err    = xy_target.getY() - currentPose.getY();
        Double  xy_err   = Math.hypot(x_err, y_err);
        boolean at_xy    = xy_err < 0.01;
        Double  velocity = 0.0;
        xy_setpoint = new TrapezoidProfile.State(-xy_err, xy_setpoint.velocity);

        if (!at_xy) {
            xy_setpoint = xy_profile.calculate(kDt, xy_setpoint, xy_goal);
            velocity    = xy_setpoint.velocity;                                                   // + xy_pid.calculate(
                                                                                                  // ( xy_last - xy_err
                                                                                                  // ) / kDt ); test the
                                                                                                  // rest first
            xy_speed    = new Translation2d(velocity * x_err / xy_err, velocity * y_err / xy_err);
        } else {
            xy_setpoint = new TrapezoidProfile.State();
            xy_speed    = new Translation2d();
        }
        return at_xy;
    }

    /**
     * Set rotation speed for swerve drive base on angle to target
     *
     * @param current_pose of the robot
     * @return boolean true if within deadband otherwise false
     */
    private boolean setRotationSpeedFromTarget(Rotation2d rotation) {
        Double  rotationSpeedDeltaToTarget    = MathUtil.angleModulus(r_target.getRadians() - rotation.getRadians());
        boolean rotationWithinAcceptableRange = Math.abs(rotationSpeedDeltaToTarget) < 0.01;

        // If the rotation is within an acceptable range, we can reset the trapezoid
        // profile state,
        // set the rotation speed to 0, and return true
        if (rotationWithinAcceptableRange) {
            r_setpoint = new TrapezoidProfile.State();
            r_speed    = 0.0;
            return true;
        }

        // The rotation is not within a range - calculate the new setpoint and speed and
        // return false
        var preSetpointProfile = new TrapezoidProfile.State(-rotationSpeedDeltaToTarget, r_setpoint.velocity);
        r_setpoint = r_profile.calculate(kDt, preSetpointProfile, r_goal);
        r_speed    = r_setpoint.velocity;
        // + r_pid.calculate( ( r_last - r_err ) / kDt );
        // test the rest first
        return false;
    }
}
