package xin.bbtt.Block;

import org.jetbrains.annotations.NotNull;

import java.util.Map;
import java.util.stream.Collectors;

public record BlockState(String blockName, int stateId, Map<String, String> properties, String boundingBox) {

    public String getProperty(String key) {
        return properties.get(key);
    }

    /**
     * Checks if the block is passable (can be walked through).
     * @return true if the block has no collision box.
     */
    public boolean isPassable() {
        if (blockName == null) return true;
        
        // Explicit air check
        if (blockName.contains("air")) return true;
        
        // If it's explicitly marked as an empty bounding box, it's passable
        if ("empty".equals(boundingBox)) return true;
        
        // If the boundingBox field is MISSING (null), we have to be careful.
        // In this dataset, most solid blocks ARE marked as "block".
        // If it's null, it's likely a non-solid block or air that didn't have the field.
        if (boundingBox == null) return true;

        return false;
    }

    /**
     * Checks if the block is solid (can be stood upon).
     */
    public boolean isSolid() {
        return "block".equals(boundingBox);
    }

    /**
     * Checks if the block is a liquid (water or lava).
     */
    public boolean isLiquid() {
        return "water".equals(blockName) || "lava".equals(blockName);
    }

    @Override
    public @NotNull String toString() {
        String props = properties.entrySet().stream()
                .map(e -> e.getKey() + "=" + e.getValue())
                .collect(Collectors.joining(","));

        return String.format("%s[%s] (id=%d, bbox=%s)", blockName, props, stateId, boundingBox);
    }
}
