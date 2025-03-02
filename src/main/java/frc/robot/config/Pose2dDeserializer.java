package frc.robot.config;

import java.io.IOException;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.JsonDeserializer;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Pose2dDeserializer extends JsonDeserializer<Pose2d> {
    @Override
    public Pose2d deserialize(JsonParser parser, DeserializationContext context) throws IOException {
        ObjectMapper mapper   = (ObjectMapper) parser.getCodec();
        JsonNode     node     = mapper.readTree(parser);

        double       x        = node.get("x").asDouble();
        double       y        = node.get("y").asDouble();
        double       rotation = node.get("rotation").asDouble();

        Pose2d       pose     = new Pose2d(x, y, new Rotation2d(Math.toRadians(rotation)));
        return pose;
    }
}