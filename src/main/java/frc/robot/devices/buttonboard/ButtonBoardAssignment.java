package frc.robot.devices.buttonboard;

public class ButtonBoardAssignment {
    /**
     * The device ID of the controller associated with this button
     */
    public int controllerId;

    /**
     * The device ID of the button associated with this button
     */
    public int buttonId;

    public ButtonBoardAssignment() {

    }

    public ButtonBoardAssignment(int controllerId, int buttonId) {
        this.controllerId = controllerId;
        this.buttonId     = buttonId;
    }
}