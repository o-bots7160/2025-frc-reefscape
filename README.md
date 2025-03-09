# 2025-frc-reefscape

A repository for assets related to the 2025 FIRST Robotics Competition REEFSCAPE
challenge for the Ludington O-Bots team 7160.

## Getting Started

- Reference the documentation found at
  [WPILib's Zero to Robot guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/introduction.html).
- Install the appropriate
  [WPILib for your OS](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html).
  Be sure to choose "Everything" when asked what to install, and then use the
  "Install for this User" button. When prompted for the VS Code Download, choose
  "Download for this computer only (fastest)"..

## Robot Source Code Structure

The source code for the robot is organized under `src/main/java/frc/robot`. Below is an overview of the folder structure:

- `commands`: Contains command classes that define the robot's actions.
- `config`: Contains simple classes that map to configuration files in the `deploy` folder, as well as helpers to consume configuration.
- `devices`: Custom classes that wrap existing device structures to make them easier to work with within this solution.
- `helpers`: Utility classes.
- `subsystems`: Contains subsystem classes that represent different parts of the robot (e.g., drivetrain, arm).
- `Robot.java`: The main class that initializes and runs the robot.
- `RobotContainer.java`: Sets up the robot's subsystems and commands, and binds them to the operator interface.

## Other Resources

- [Limelight Smart Camera | FRC Programming](https://docs.limelightvision.io/docs/docs-limelight/getting-started/FRC/programming)

