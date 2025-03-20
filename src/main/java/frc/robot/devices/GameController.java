package frc.robot.devices;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GameController extends CommandJoystick {

    /**
     * Enum representing the buttons on a game controller. Each button is associated with a specific integer value.
     */
    public enum GameControllerButton {
        A(1), B(2), X(3), Y(4), L1(5), R1(6), Back(7), Start(8), LStick(9), RStick(10);

        private final int value;

        GameControllerButton(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    /**
     * Represents the axes available on a game controller, providing clear mappings between logical axis names and their corresponding numeric
     * identifiers.
     */
    public enum GameControllerAxes {
        LeftStickX(0), LeftStickY(1), RightStickX(4), RightStickY(5), LTrigger(2), RTrigger(3);

        private final int value;

        GameControllerAxes(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public GameController(int port) {
        super(port);
    }

    /**
     * Assigns a command to be executed while a specified button is held down.
     *
     * @param button           The button to which the command will be assigned.
     * @param whileHeldCommand The command to be executed while the button is held down.
     * @return A Trigger object that represents the button hold condition.
     */
    public Trigger onButtonHold(GameControllerButton button, Command whileHeldCommand) {
        return new Trigger(button(button.getValue())).whileTrue(whileHeldCommand);
    }

    /**
     * Assigns a command to be executed while a specified button is held down.
     *
     * @param button              The button to which the command will be assigned.
     * @param whileHeldCommand    The command to be executed while the button is held down.
     * @param whenReleasedCommand The command to be executed when the button is released.
     * @return A Trigger object that represents the button hold condition.
     */
    public Trigger onButtonHold(GameControllerButton button, Command whileHeldCommand, Command whenReleasedCommand) {
        return new Trigger(button(button.getValue())).whileTrue(whileHeldCommand).onFalse(whenReleasedCommand);
    }

    /**
     * Assigns a command to be executed when a specific button on the game controller is pressed.
     *
     * @param button         The button on the game controller to which the command will be assigned.
     * @param onPressCommand The command to be executed when the button is pressed.
     * @return A Trigger object that represents the button press event.
     */
    public Trigger onButtonPress(GameControllerButton button, Command onPressCommand) {
        return new Trigger(button(button.getValue())).onTrue(onPressCommand);
    }

    /**
     * Assigns commands to be executed when a specific button is held down and then released.
     *
     * @param button           The button on the game controller to monitor.
     * @param onPressCommand   The command to execute when the button is pressed.
     * @param onReleaseCommand The command to execute when the button is released.
     * @return A Trigger object that monitors the button state and executes the appropriate commands.
     */
    public Trigger onButtonPress(GameControllerButton button, Command onPressCommand, Command onReleaseCommand) {
        return onButtonPress(button, onPressCommand).onFalse(onReleaseCommand);
    }

    public Trigger onAxisChange(GameControllerAxes axis, Function<Double, Boolean> conditionFunction, Command whileTrueCommand) {
        Double axisValue = getRawAxis(axis.getValue());

        return new Trigger(() -> conditionFunction.apply(axisValue)).whileTrue(whileTrueCommand);

    }

    public Trigger onAxisChange(GameControllerAxes axis, Function<Double, Boolean> conditionFunction, Command whileTrueCommand,
            Command whileFalseCommand) {
        return onAxisChange(axis, conditionFunction, whileTrueCommand).onFalse(whileFalseCommand);
    }
}
