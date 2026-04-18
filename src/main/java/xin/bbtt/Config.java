package xin.bbtt;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Properties;

public class Config {
    private static final String CONFIG_FILE = "plugin/MovementSync/config.properties";
    public static boolean noFall = false;

    public static void load() {
        Properties props = new Properties();
        File file = new File(CONFIG_FILE);
        if (file.exists()) {
            try (FileInputStream in = new FileInputStream(file)) {
                props.load(in);
                noFall = Boolean.parseBoolean(props.getProperty("nofall", "true"));
            } catch (IOException e) {
                MovementSync.Instance.getLogger().error("Failed to load config file", e);
            }
        } else {
            save(); // Create with defaults
        }
    }

    public static void save() {
        Properties props = new Properties();
        props.setProperty("nofall", String.valueOf(noFall));
        
        File file = new File(CONFIG_FILE);
        if (file.getParentFile() != null) {
            file.getParentFile().mkdirs();
        }
        try (FileOutputStream out = new FileOutputStream(file)) {
            props.store(out, "MovementSync Configuration");
        } catch (IOException e) {
            MovementSync.Instance.getLogger().error("Failed to save config file", e);
        }
    }
}
