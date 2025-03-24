package frc.robot.devices.buttonboard;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.devices.GameController;

public class ButtonBoardController<TButtonBoardLayout extends IButtonBoardButton> {

    private Map<Integer, GameController> gameControllers = new HashMap<Integer, GameController>();

    public ButtonBoardController(int... ports) {
        for (int i : ports) {
            gameControllers.put(i, new GameController(i));
        }
    }

    /**
     * Assigns a command to be executed while a specified button is held down.
     *
     * @param button           The button to which the command will be assigned.
     * @param whileHeldCommand The command to be executed while the button is held down.
     * @return A Trigger object that represents the button hold condition.
     */
    public Trigger onButtonHold(TButtonBoardLayout button, Command whileHeldCommand) {
        var buttonBoardAssignment = button.getValue();
        var gameController        = gameControllers.get(buttonBoardAssignment.controllerId);

        return gameController.onButtonHold(buttonBoardAssignment.buttonId, whileHeldCommand);
    }

    /**
     * Assigns a command to be executed when a specific button on the game controller is pressed.
     *
     * @param button         The button on the game controller to which the command will be assigned.
     * @param onPressCommand The command to be executed when the button is pressed.
     * @return A Trigger object that represents the button press event.
     */
    public Trigger onButtonPress(TButtonBoardLayout button, Command onPressCommand) {
        var buttonBoardAssignment = button.getValue();
        var gameController        = gameControllers.get(buttonBoardAssignment.controllerId);

        return gameController.onButtonPress(buttonBoardAssignment.buttonId, onPressCommand);
    }

    /**
     * Assigns commands to be executed when a specific button is held down and then released.
     *
     * @param button           The button on the game controller to monitor.
     * @param onPressCommand   The command to execute when the button is pressed.
     * @param onReleaseCommand The command to execute when the button is released.
     * @return A Trigger object that monitors the button state and executes the appropriate commands.
     */
    public Trigger onButtonPress(TButtonBoardLayout button, Command onPressCommand, Command onReleaseCommand) {
        var buttonBoardAssignment = button.getValue();
        var gameController        = gameControllers.get(buttonBoardAssignment.controllerId);

        return gameController.onButtonPress(buttonBoardAssignment.buttonId, onPressCommand, onReleaseCommand);
    }

    /**
     * Gets the current state of the {@link TButtonBoardLayout}
     * 
     * @param button the button to check
     * @return if the button is pressed, returns true; else false
     */
    public boolean isPressed(TButtonBoardLayout button) {
        var buttonBoardAssignment = button.getValue();
        var gameController        = gameControllers.get(buttonBoardAssignment.controllerId);

        return gameController.button(buttonBoardAssignment.buttonId).getAsBoolean();
    }

}
