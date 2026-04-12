package xin.bbtt.Block;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import lombok.Getter;
import org.jetbrains.annotations.NotNull;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.*;
import java.util.stream.Collectors;

public class BlockStateParser {
    private final TreeMap<Integer, BlockEntry> rangeMap = new TreeMap<>();
    @Getter
    private static int blockStateRegistrySize = 0;
    public static BlockStateParser Instance = new BlockStateParser();

    private BlockStateParser() {}

    static {
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(
                Objects.requireNonNull(BlockStateParser.class.getClassLoader().getResourceAsStream("blocks.json")),
                StandardCharsets.UTF_8))) {
            String content = reader.lines().collect(Collectors.joining("\n"));
            Instance.loadJson(content);
        } catch (Exception e) {
            throw new RuntimeException("Failed to load blocks.json", e);
        }
    }

    public void loadJson(String jsonContent) {
        int maxId = -1;
        Gson gson = new Gson();
        List<BlockEntry> entries = gson.fromJson(jsonContent, new TypeToken<List<BlockEntry>>(){}.getType());
        for (BlockEntry entry : entries) {
            rangeMap.put(entry.getMinStateId(), entry);
            if (entry.getMaxStateId() > maxId) {
                maxId = entry.getMaxStateId();
            }
        }
        blockStateRegistrySize = maxId + 1;
    }

    public BlockState parseStateId(int stateId) {
        Map.Entry<Integer, BlockEntry> floorEntry = rangeMap.floorEntry(stateId);

        if (floorEntry == null || stateId > floorEntry.getValue().getMaxStateId()) {
            throw new UnknownBlockStateException(stateId);
        }

        BlockEntry block = floorEntry.getValue();
        Map<String, String> properties = getProperties(stateId, block);

        return new BlockState(block.getName(), stateId, properties);
    }

    private static @NotNull Map<String, String> getProperties(int stateId, BlockEntry block) {
        int offset = stateId - block.getMinStateId();

        Map<String, String> properties = new LinkedHashMap<>();
        List<BlockEntry.StateProperty> states = block.getStates();

        if (states != null && !states.isEmpty()) {
            for (int i = states.size() - 1; i >= 0; i--) {
                BlockEntry.StateProperty prop = states.get(i);
                int numValues = prop.getNum_values();

                int valueIndex = offset % numValues;
                properties.put(prop.getName(), prop.getValueAt(valueIndex));

                offset /= numValues;
            }
        }
        return properties;
    }
}
