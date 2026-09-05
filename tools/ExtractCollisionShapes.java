import java.io.BufferedWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import net.minecraft.SharedConstants;
import net.minecraft.core.BlockPos;
import net.minecraft.core.registries.BuiltInRegistries;
import net.minecraft.server.Bootstrap;
import net.minecraft.world.level.EmptyBlockGetter;
import net.minecraft.world.level.block.Block;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.block.state.properties.Property;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.shapes.CollisionContext;
import net.minecraft.world.phys.shapes.VoxelShape;

/**
 * Dumps Paper's authoritative collision VoxelShape for every protocol block-state ID.
 * This source is launched directly with a Java 21+ runtime by generate_collision_shapes.py.
 */
public final class ExtractCollisionShapes {
    private static String properties(BlockState state) {
        List<String> values = new ArrayList<>();
        for (Map.Entry<Property<?>, Comparable<?>> entry : state.getValues().entrySet()) {
            @SuppressWarnings("rawtypes")
            Property property = entry.getKey();
            values.add(property.getName() + "=" + property.getName(entry.getValue()));
        }
        values.sort(Comparator.naturalOrder());
        return String.join(",", values);
    }

    private static String boxes(BlockState state) {
        VoxelShape shape = state.getCollisionShape(
            EmptyBlockGetter.INSTANCE, BlockPos.ZERO, CollisionContext.empty());
        List<String> values = new ArrayList<>();
        for (AABB box : shape.toAabbs()) {
            values.add(box.minX + "," + box.minY + "," + box.minZ + ","
                + box.maxX + "," + box.maxY + "," + box.maxZ);
        }
        return String.join(";", values);
    }

    public static void main(String[] args) throws Exception {
        if (args.length != 1) throw new IllegalArgumentException("output TSV required");
        SharedConstants.tryDetectVersion();
        Bootstrap.bootStrap();
        try (BufferedWriter out = Files.newBufferedWriter(Path.of(args[0]))) {
            out.write("#registry_size=" + Block.BLOCK_STATE_REGISTRY.size());
            out.newLine();
            for (int id = 0; id < Block.BLOCK_STATE_REGISTRY.size(); id++) {
                BlockState state = Block.stateById(id);
                String name = BuiltInRegistries.BLOCK.getKey(state.getBlock()).toString();
                out.write(id + "\t" + name + "\t" + properties(state) + "\t" + boxes(state));
                out.newLine();
            }
        }
    }
}
