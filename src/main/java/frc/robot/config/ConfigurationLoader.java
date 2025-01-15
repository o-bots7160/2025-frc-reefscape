package frc.robot.config;

import java.io.File;
import java.io.IOException;

import com.fasterxml.jackson.core.exc.StreamReadException;
import com.fasterxml.jackson.databind.DatabindException;
import com.fasterxml.jackson.databind.JavaType;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.type.TypeFactory;

import edu.wpi.first.wpilibj.Filesystem;

public class ConfigurationLoader {

    public static <TConfig> TConfig load(String fileName, Class<TConfig> classOfT)
            throws StreamReadException, DatabindException, IOException {
        // Generic and Mapping Setup
        JavaType type = TypeFactory.defaultInstance().constructType(classOfT);
        ObjectMapper om = new ObjectMapper();

        // File setup
        File deployDirectory = Filesystem.getDeployDirectory();
        File configFile = new File(deployDirectory, fileName);

        // Map the config to the class type and return
        TConfig config = om.readValue(configFile, type);

        return config;
    }
}
