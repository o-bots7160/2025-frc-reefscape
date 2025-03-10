package frc.robot.devices;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBoardController {
    /**
     * Enum representing the buttons on the button board. Each button is associated with a specific controller id and integer value
     */
    public enum ReefLevel {
        L1, L2, L3, L4
    }

    public enum GamePiece {
        Algae, Coral
    }

    public enum ButtonBoardButton {
        // Levels
        L1(3, 5), L2(3, 8), L3(3, 7), L4(2, 8),
        // Reef Positions (Dial)
        A(2, 7), B(1, 7), C(1, 8), D(1, 4), E(1, 6), F(1, 5), G(2, 4), H(2, 5), I(2, 1), J(2, 3), K(2, 6), L(1, 1),
        // Switch
        Switch(3, 4),
        // Climbing
        ClimbUp(1, 3), ClimbDown(3, 6),
        // Locations
        Net(2, 2), Processor(1, 2), CoralStation(3, 3),
        // Actions
        Eject(3, 1), Travel(3, 2);

        private final int value;

        private final int controller;

        ButtonBoardButton(int controller, int value) {
            this.controller = controller;
            this.value      = value;
        }

        public int[] getValue() {
            return new int[] { controller - 1, value - 1 };
        }
    }

    private GameController[] gameControllers;

    public ButtonBoardController(int port1, int port2, int port3, int port4) {
        gameControllers = new GameController[] { new GameController(port1), new GameController(port2), new GameController(port3),
                new GameController(port4) };
    }

    /**
     * Assigns a command to be executed while a specified button is held down.
     *
     * @param button           The button to which the command will be assigned.
     * @param whileHeldCommand The command to be executed while the button is held down.
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
     * Assigns a command to be executed when a specific button on the game controller is pressed.
     *
     * @param button         The button on the game controller to which the command will be assigned.
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
     * Assigns commands to be executed when a specific button is held down and then released.
     *
     * @param button           The button on the game controller to monitor.
     * @param onPressCommand   The command to execute when the button is pressed.
     * @param onReleaseCommand The command to execute when the button is released.
     * @return A Trigger object that monitors the button state and executes the appropriate commands.
     */
    public Trigger onButtonPress(ButtonBoardButton button, Command onPressCommand, Command onReleaseCommand) {
        var buttonValues   = button.getValue();
        var controllerId   = buttonValues[0];
        var buttonId       = GameController.GameControllerButton.values()[buttonValues[1]];
        var gameController = gameControllers[controllerId];

        return gameController.onButtonPress(buttonId, onPressCommand, onReleaseCommand);
    }

    /**
     * Gets the current state of the {@link ButtonBoardButton}
     * 
     * @param button the button to check
     * @return if the button is pressed, returns true; else false
     */
    public boolean isPressed(ButtonBoardButton button) {
        var buttonValues = button.getValue();
        return gameControllers[buttonValues[0]].button(buttonValues[1]).getAsBoolean();
    }

    ReefLevel getReefLevel() {
        ReefLevel level = ReefLevel.L4;
        if (isPressed(ButtonBoardButton.L1)) {
            level = ReefLevel.L1;
        }
        if (isPressed(ButtonBoardButton.L2)) {
            level = ReefLevel.L2;
        }
        if (isPressed(ButtonBoardButton.L3)) {
            level = ReefLevel.L3;
        }
        if (isPressed(ButtonBoardButton.L4)) {
            level = ReefLevel.L4;
        }
        return level;
    }

    GamePiece getGamePiece() {
        GamePiece piece = GamePiece.Algae;
        if (isPressed(ButtonBoardButton.Switch)) {
            piece = GamePiece.Coral;
        }
        return piece;
    }
}
