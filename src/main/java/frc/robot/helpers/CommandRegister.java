package frc.robot.helpers;

import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.CommandFactory;
import frc.robot.config.AllianceLandmarkConfig;

public class CommandRegister {

    private class AlgaeReefMapping {

        public String name;

        public Pose2d position;

        public Double height;

        public Double rotation;

        public AlgaeReefMapping(String name, Pose2d position, Double height, Double rotation) {
            this.name     = name;
            this.position = position;
            this.rotation = rotation;
            this.height   = height;

        }
    }

    private CommandFactory         cf;

    private AllianceLandmarkConfig landmarks;

    public CommandRegister(AllianceLandmarkConfig landmarks, CommandFactory commandFactory) {
        this.landmarks = landmarks;
        this.cf        = commandFactory;

        init();
    }

    public void init() {
        // Coral Commands
        Map<String, Pose2d> reefZones = Map.ofEntries(
                Map.entry("A", landmarks.reefZoneA),
                Map.entry("B", landmarks.reefZoneB),
                Map.entry("C", landmarks.reefZoneC),
                Map.entry("D", landmarks.reefZoneD),
                Map.entry("E", landmarks.reefZoneE),
                Map.entry("F", landmarks.reefZoneF),
                Map.entry("G", landmarks.reefZoneG),
                Map.entry("H", landmarks.reefZoneH),
                Map.entry("I", landmarks.reefZoneI),
                Map.entry("J", landmarks.reefZoneJ),
                Map.entry("K", landmarks.reefZoneK),
                Map.entry("L", landmarks.reefZoneL));

        for (Map.Entry<String, Pose2d> entry : reefZones.entrySet()) {
            String zone     = entry.getKey();
            Pose2d reefZone = entry.getValue();
            for (int level = 1; level <= 4; level++) {
                double coralLevel    = switch (level) {
                                     case 1 -> landmarks.coralLevel1;
                                     case 2 -> landmarks.coralLevel2;
                                     case 3 -> landmarks.coralLevel3;
                                     case 4 -> landmarks.coralLevel4;
                                     default -> throw new IllegalStateException("Unexpected value: " + level);
                                     };
                double coralRotation = switch (level) {
                                     case 1 -> landmarks.coralLevel1Rotation;
                                     case 2 -> landmarks.coralLevel2Rotation;
                                     case 3 -> landmarks.coralLevel3Rotation;
                                     case 4 -> landmarks.coralLevel4Rotation;
                                     default -> throw new IllegalStateException("Unexpected value: " + level);
                                     };

                // This will register a command with a name like "Place Coral A1"
                NamedCommands.registerCommand("Place Coral " + zone + level,
                        cf.createPlaceCoralCommand(zone, String.valueOf(level), () -> reefZone, () -> coralLevel, () -> coralRotation));
            }
        }

        // Algae Commands
        Map<String, AlgaeReefMapping> reefFaces = Map.ofEntries(
                Map.entry("AB", new AlgaeReefMapping("High", landmarks.reefFaceAB, landmarks.algaeHigh, landmarks.algaeHighRotation)),
                Map.entry("CD", new AlgaeReefMapping("Low", landmarks.reefFaceCD, landmarks.algaeLow, landmarks.algaeLowRotation)),
                Map.entry("EF", new AlgaeReefMapping("High", landmarks.reefFaceEF, landmarks.algaeHigh, landmarks.algaeHighRotation)),
                Map.entry("GH", new AlgaeReefMapping("Low", landmarks.reefFaceGH, landmarks.algaeLow, landmarks.algaeLowRotation)),
                Map.entry("IJ", new AlgaeReefMapping("High", landmarks.reefFaceIJ, landmarks.algaeHigh, landmarks.algaeHighRotation)),
                Map.entry("KL", new AlgaeReefMapping("Low", landmarks.reefFaceKL, landmarks.algaeLow, landmarks.algaeLowRotation)));

        for (Map.Entry<String, AlgaeReefMapping> entry : reefFaces.entrySet()) {
            String           face    = entry.getKey();
            AlgaeReefMapping mapping = entry.getValue();

            // This will register a command with a name like "Place Algae AB"
            NamedCommands.registerCommand("Place Algae " + face,
                    cf.createTakeAlgaeCommand(face, mapping.name, () -> mapping.position, () -> mapping.height, () -> mapping.rotation));

            NamedCommands.registerCommand("Move to Reef Face " + face, cf.createDriveBaseMoveToCommand(mapping.position));
        }

        // Other Commands
        NamedCommands.registerCommand("Travel Command", cf.createTravelCommand());
        NamedCommands.registerCommand("Coral Station Command", cf.createCoralStationCommand());
        NamedCommands.registerCommand("Net Command", cf.createNetCommand());
        NamedCommands.registerCommand("Processor Command", cf.createProcessorCommand());
        NamedCommands.registerCommand("Eject Coral Command", cf.createEjectCoralCommand());
        NamedCommands.registerCommand("Eject Algae Command", cf.createEjectAlgaeCommand());
    }

}
