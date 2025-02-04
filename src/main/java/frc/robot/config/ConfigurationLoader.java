package frc.robot.config;

import java.io.File;
import javax.naming.ConfigurationException;

import com.fasterxml.jackson.databind.JavaType;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.type.TypeFactory;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Filesystem;

@Logged
public class ConfigurationLoader {

    /**
     * Loads a configuration file from the deploy directory and maps it to a type
     *
     * @param <TConfig> The Java type to map the configuration file
     * @param fileName  The name of the JSON file to load
     * @param classOfT  The class type to map the configuration file to
     * @return
     * @throws ConfigurationException
     */
    public static <TConfig> TConfig load(String fileName, Class<TConfig> classOfT) throws ConfigurationException {
        try {
            // Generic and Mapping Setup
            JavaType     type            = TypeFactory.defaultInstance().constructType(classOfT);
            ObjectMapper om              = new ObjectMapper();

            // File setup
            File         deployDirectory = Filesystem.getDeployDirectory();
            File         configFile      = new File(deployDirectory, fileName);

            // Map the config to the class type and return
            TConfig      config          = om.readValue(configFile, type);

            return config;
        } catch (Exception e) {
            e.printStackTrace();

            throw new ConfigurationException("Failed to load configuration file: " + fileName);
        }
    }
}
