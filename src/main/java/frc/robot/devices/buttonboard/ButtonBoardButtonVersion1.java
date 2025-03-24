package frc.robot.devices.buttonboard;

public enum ButtonBoardButtonVersion1 implements IButtonBoardButton {
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

    ButtonBoardButtonVersion1(int controller, int value) {
        this.controller = controller;
        this.value      = value;
    }

    ButtonBoardButtonVersion1(ButtonBoardAssignment buttonBoardAssignment) {
        this(buttonBoardAssignment.controllerId, buttonBoardAssignment.buttonId);
    }

    @Override
    public ButtonBoardAssignment getValue() {
        return new ButtonBoardAssignment(controller, value);
    }
}