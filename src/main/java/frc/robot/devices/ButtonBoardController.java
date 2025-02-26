package frc.robot.devices;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBoardController{
    /**
     * Enum representing the buttons on the button board. Each button is associated
     * with a specific controller id and integer value
     */
    public enum ButtonBoardButton {
        // Levels
        L1(0, 7), L2(0, 8), L3(1, 7), L4(1, 8),
        // Reef Positions (Dial)
        A(3, 7), B(1, 4), C(1, 3), D(2, 4), E(2, 6), F(2, 5), G(3, 4), H(3, 5), I(3, 8), J(3, 3), K(3, 6), L(2, 7),
        // Switch
        Switch(0, 4),
        // Climbing
        ClimbUp(2, 3), ClimbDown(0, 6),
        // Locations
        Net(1, 6), Processor(1, 1), CoralStation(0, 3),
        // Actions
        Place(1, 2), Travel(0, 2), Lock(0, 5);

        private final int value;

        private final int controller;

        ButtonBoardButton(int controller, int value) {
            this.controller = controller;
            this.value      = value;
        }

        public int[] getValue() {
            return new int[] { controller, value };
        }
    }

    private GameController[] gameControllers;

    public ButtonBoardController(int port1, int port2, int port3, int port4) {
        gameControllers = new GameController[] { new GameController(port1), new GameController(port2),
                new GameController(port3), new GameController(port4) };
    }

    /**
     * Assigns a command to be executed while a specified button is held down.
     *
     * @param button           The button to which the command will be assigned.
     * @param whileHeldCommand The command to be executed while the button is held
     *                         down.
     * @return A Trigger object that represents the button hold condition.
     */
    public Trigger onButtonHold(ButtonBoardButton button, Command whileHeldCommand) {
        var buttonValues   = button.getValue();
        var controllerId   = buttonValues[0];
        var buttonId       = GameController.GameControllerButton.values()[buttonValues[1]];
        var gameController = gameControllers[controllerId];

        return gameController.onButtonHold(buttonId, whileHeldCommand);
    }

    /**
     * Assigns a command to be executed when a specific button on the game
     * controller is pressed.
     *
     * @param button         The button on the game controller to which the command
     *                       will be assigned.
     * @param onPressCommand The command to be executed when the button is pressed.
     * @return A Trigger object that represents the button press event.
     */
    public Trigger onButtonPress(ButtonBoardButton button, Command onPressCommand) {
        var buttonValues   = button.getValue();
        var controllerId   = buttonValues[0];
        var buttonId       = GameController.GameControllerButton.values()[buttonValues[1]];
        var gameController = gameControllers[controllerId];

        return gameController.onButtonPress(buttonId, onPressCommand);
    }

    /**
     * Assigns commands to be executed when a specific button is held down and then
     * released.
     *
     * @param button           The button on the game controller to monitor.
     * @param onPressCommand   The command to execute when the button is pressed.
     * @param onReleaseCommand The command to execute when the button is released.
     * @return A Trigger object that monitors the button state and executes the
     *         appropriate commands.
     */
    public Trigger onButtonPress(ButtonBoardButton button, Command onPressCommand, Command onReleaseCommand) {
        var buttonValues   = button.getValue();
        var controllerId   = buttonValues[0];
        var buttonId       = GameController.GameControllerButton.values()[buttonValues[1]];
        var gameController = gameControllers[controllerId];

        return gameController.onButtonPress(buttonId, onPressCommand, onReleaseCommand);
    }

    public boolean isPressed(ButtonBoardButton button)
    {
        int [] location = button.getValue();
        return gameControllers[ location[0]].button( location[1]).getAsBoolean();
    }
}
