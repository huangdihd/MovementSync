package xin.bbtt.inventory;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import lombok.Data;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.stream.Collectors;

public class EnchantmentRegistry {
    private final Map<Integer, EnchantmentEntry> idToEnchantment = new HashMap<>();
    private final Map<String, EnchantmentEntry> nameToEnchantment = new HashMap<>();
    
    public static EnchantmentRegistry Instance = new EnchantmentRegistry();

    private EnchantmentRegistry() {
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(
                Objects.requireNonNull(EnchantmentRegistry.class.getClassLoader().getResourceAsStream("enchantments.json")),
                StandardCharsets.UTF_8))) {
            loadJson(reader.lines().collect(Collectors.joining("\n")));
        } catch (Exception e) {
            // Log error or handle gracefully
        }
    }

    public void loadJson(String jsonContent) {
        Gson gson = new Gson();
        List<EnchantmentEntry> entries = gson.fromJson(jsonContent, new TypeToken<List<EnchantmentEntry>>(){}.getType());
        for (EnchantmentEntry entry : entries) {
            idToEnchantment.put(entry.getId(), entry);
            nameToEnchantment.put(entry.getName(), entry);
        }
    }

    public EnchantmentEntry getEnchantment(int id) {
        return idToEnchantment.get(id);
    }

    public EnchantmentEntry getEnchantment(String name) {
        return nameToEnchantment.get(name);
    }

    public EnchantmentEntry getByNetworkId(int networkId) {
        String name = xin.bbtt.listeners.RegistryDataListener.getNetworkIdToEnchantmentName().get(networkId);
        if (name == null) return null;
        // Strip namespace if needed (e.g. minecraft:efficiency -> efficiency)
        if (name.contains(":")) name = name.split(":")[1];
        return nameToEnchantment.get(name);
    }

    @Data
    public static class EnchantmentEntry {
        private int id;
        private String name;
        private String displayName;
        private int maxLevel;
        private boolean treasureOnly;
        private boolean curse;
        private String category;
        private int weight;
    }
}
