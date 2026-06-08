package xin.bbtt.Entity;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import lombok.Data;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class EntityRegistry {
    private final Map<String, EntityEntry> nameToEntity = new HashMap<>();
    
    public static EntityRegistry Instance = new EntityRegistry();

    private EntityRegistry() {
        try (InputStream is = EntityRegistry.class.getClassLoader().getResourceAsStream("entities.json")) {
            if (is != null) {
                try (BufferedReader reader = new BufferedReader(new InputStreamReader(is, StandardCharsets.UTF_8))) {
                    loadJson(reader.lines().collect(Collectors.joining("\n")));
                }
            }
        } catch (Exception e) {
            xin.bbtt.MovementSync.getLogger().error("Failed to load entities.json registry", e);
        }
    }

    public void loadJson(String jsonContent) {
        Gson gson = new Gson();
        List<EntityEntry> entries = gson.fromJson(jsonContent, new TypeToken<List<EntityEntry>>(){}.getType());
        for (EntityEntry entry : entries) {
            nameToEntity.put(entry.getName().toLowerCase(), entry);
        }
    }

    public EntityEntry getEntity(String name) {
        if (name == null) return null;
        return nameToEntity.get(name.toLowerCase());
    }

    @Data
    public static class EntityEntry {
        private int id;
        private String name;
        private float width;
        private float height;
        private List<String> metadataKeys;
    }
}
