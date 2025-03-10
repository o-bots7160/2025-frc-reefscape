package frc.robot.helpers;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.CommandFactory;
import frc.robot.config.AllianceLandmarkConfig;

public class CommandRegister {

    private CommandFactory            cf;
    private AllianceLandmarkConfig    landmarks;
    private TriggerBindings           triggerBindings;

    public CommandRegister(AllianceLandmarkConfig landmarks, CommandFactory commandFactory, TriggerBindings triggerBindings)
    {
        this.landmarks          = landmarks;
        this.cf                 = commandFactory;
        this.triggerBindings    = triggerBindings;

        init();
    }

    public void init()
    {
        NamedCommands.registerCommand("Place Coral L1",          triggerBindings.createLevelSelectCommand(landmarks.coralLevel1, landmarks.coralLevel1Rotation));
        NamedCommands.registerCommand("Place Coral L2",          triggerBindings.createLevelSelectCommand(landmarks.coralLevel2, landmarks.coralLevel2Rotation));
        NamedCommands.registerCommand("Place Coral L3",          triggerBindings.createLevelSelectCommand(landmarks.coralLevel3, landmarks.coralLevel3Rotation));
        NamedCommands.registerCommand("Place Coral L4",          triggerBindings.createLevelSelectCommand(landmarks.coralLevel4, landmarks.coralLevel4Rotation));
        NamedCommands.registerCommand("Algae High Command",      triggerBindings.createLevelSelectCommand(landmarks.algaeHigh, landmarks.algaeHighRotation));
        NamedCommands.registerCommand("Algae Low Command",       triggerBindings.createLevelSelectCommand(landmarks.algaeLow, landmarks.algaeLowRotation));
        NamedCommands.registerCommand("Travel Command",          cf.createTravelCommand());
        NamedCommands.registerCommand("Coral Station Command",   cf.createCoralStationCommand());
        NamedCommands.registerCommand("Net Command",             cf.createNetCommand());
        NamedCommands.registerCommand("Processor Command",       cf.createProcessorCommand());
        NamedCommands.registerCommand("Eject Coral Command",     cf.createEjectCoralCommand());
        NamedCommands.registerCommand("Eject Algae Command",     cf.createEjectAlgaeCommand());
    }

}
