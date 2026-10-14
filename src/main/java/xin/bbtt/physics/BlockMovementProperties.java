package xin.bbtt.physics;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import xin.bbtt.Block.BlockState;

import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

/** Block settings and collision behaviors from vanilla 1.21.11; unknown blocks use ordinary ground. */
public record BlockMovementProperties(double slipperiness, double velocityMultiplier, double jumpMultiplier,
                                      double bounce, boolean stickyStepping, boolean honeySliding,
                                      double stuckHorizontal, double stuckVertical,
                                      boolean bubbleColumn, boolean ignoresSupportVelocity, boolean requiresFeetContact) {
    // Keep the established ordinary-ground trajectory bit-compatible with callers.
    public static final BlockMovementProperties DEFAULT = new BlockMovementProperties(
            0.6, 1, 1, 0, false, false, 1, 1, false, false, false);
    private static final Map<String, BlockMovementProperties> BLOCKS = load();

    public static BlockMovementProperties of(BlockState state) {
        return BLOCKS.getOrDefault(state.blockName().replace("minecraft:", ""), DEFAULT);
    }

    public boolean stuck() {
        return stuckHorizontal != 1 || stuckVertical != 1;
    }

    private static Map<String, BlockMovementProperties> load() {
        try (InputStreamReader reader = new InputStreamReader(Objects.requireNonNull(
                BlockMovementProperties.class.getClassLoader().getResourceAsStream("block_movement_properties.json"),
                "Missing block movement properties"), StandardCharsets.UTF_8)) {
            Map<String, BlockMovementProperties> properties = new HashMap<>();
            JsonParser.parseReader(reader).getAsJsonObject().entrySet().forEach(entry -> {
                JsonObject value = entry.getValue().getAsJsonObject();
                properties.put(entry.getKey(), new BlockMovementProperties(
                        number(value, "slipperiness", 0.6), number(value, "velocityMultiplier", 1),
                        number(value, "jumpMultiplier", 1), number(value, "bounce", 0),
                        flag(value, "stickyStepping"), flag(value, "honeySliding"),
                        number(value, "stuckHorizontal", 1), number(value, "stuckVertical", 1),
                        flag(value, "bubbleColumn"), flag(value, "ignoresSupportVelocity"),
                        flag(value, "requiresFeetContact")));
            });
            return Map.copyOf(properties);
        } catch (IOException error) {
            throw new ExceptionInInitializerError(error);
        }
    }

    private static double number(JsonObject value, String key, double fallback) {
        // Vanilla block settings and the non-integral slowdown constants are floats.
        return value.has(key) ? (double) value.get(key).getAsFloat() : fallback;
    }

    private static boolean flag(JsonObject value, String key) {
        return value.has(key) && value.get(key).getAsBoolean();
    }
}
