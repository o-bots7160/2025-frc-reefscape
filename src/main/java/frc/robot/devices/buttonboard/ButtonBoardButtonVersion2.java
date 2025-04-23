package frc.robot.devices.buttonboard;

public enum ButtonBoardButtonVersion2 implements IButtonBoardButton {
    // Levels
    L1(2, 8), L2(2, 7), L3(3, 7), L4(3, 8),
    // Reef Positions (Dial)
    // A(2, 7), B(1, 7), C(1, 8), D(1, 4), E(1, 6), F(1, 5), G(2, 4), H(2, 5), I(2, 1), J(2, 3), K(2, 6), L(1, 1),
    // Switch
    Switch(3, 3),
    // Climbing
    ClimbUp(3, 4), ClimbDown(2, 6),
    // Locations
    Net(3, 5), Processor(3, 2), CoralStation(3, 1),
    // Actions
    Eject(2, 4), Travel(2, 5);

    private final int value;

    private final int controller;

    ButtonBoardButtonVersion2(int controller, int value) {
        this.controller = controller;
        this.value      = value;
    }

    ButtonBoardButtonVersion2(ButtonBoardAssignment buttonBoardAssignment) {
        this(buttonBoardAssignment.controllerId, buttonBoardAssignment.buttonId);
    }

    @Override
    public ButtonBoardAssignment getValue() {
        return new ButtonBoardAssignment(controller, value);
    }
}