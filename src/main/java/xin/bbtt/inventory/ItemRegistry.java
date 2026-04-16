package xin.bbtt.inventory;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import lombok.Data;
import lombok.Getter;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.stream.Collectors;

public class ItemRegistry {
    private final Map<Integer, ItemEntry> idToItem = new HashMap<>();
    private final Map<String, ItemEntry> nameToItem = new HashMap<>();
    
    public static ItemRegistry Instance = new ItemRegistry();

    private ItemRegistry() {
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(
                Objects.requireNonNull(ItemRegistry.class.getClassLoader().getResourceAsStream("items.json")),
                StandardCharsets.UTF_8))) {
            loadJson(reader.lines().collect(Collectors.joining("\n")));
        } catch (Exception e) {
            // Log error or handle gracefully
        }
    }

    public void loadJson(String jsonContent) {
        Gson gson = new Gson();
        List<ItemEntry> entries = gson.fromJson(jsonContent, new TypeToken<List<ItemEntry>>(){}.getType());
        for (ItemEntry entry : entries) {
            idToItem.put(entry.getId(), entry);
            nameToItem.put(entry.getName(), entry);
        }
    }

    public ItemEntry getItem(int id) {
        return idToItem.get(id);
    }

    public ItemEntry getItem(String name) {
        return nameToItem.get(name);
    }

    @Data
    public static class ItemEntry {
        private int id;
        private String name;
        private String displayName;
        private int stackSize;
    }
}
