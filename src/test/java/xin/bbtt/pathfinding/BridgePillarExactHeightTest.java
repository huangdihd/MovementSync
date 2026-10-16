package xin.bbtt.pathfinding;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.inventory.ItemRegistry;
import xin.bbtt.world.World;

final class BridgePillarExactHeightTest {
    private static final int AIR = 0;
    private static final int STONE = 1;
    private static final int WHITE_CARPET = 12694;

    @AfterEach
    void resetSingleton() {
        MovementSync.INSTANCE = null;
    }

    @Test
    void bridgeRejectsPlacedBlockTooHighForStepUp() {
        BridgePillarStrategy strategy = strategyWithBlocks();

        List<Edge> edges = strategy.findEdges(new Node(0, 1, 0), carpetWorld());

        assertFalse(edges.stream().anyMatch(edge ->
            edge.getType() == BuiltinMovementType.BRIDGE
                && edge.getTarget().equals(new Node(1, 1, 0))));
    }

    @Test
    void pillarRejectsLowSupportThatCannotClearPlacedFullBlock() {
        BridgePillarStrategy strategy = strategyWithBlocks();

        List<Edge> edges = strategy.findEdges(new Node(0, 1, 0), carpetWorld());

        assertFalse(edges.stream().anyMatch(edge -> edge.getType() == BuiltinMovementType.PILLAR));
    }

    @Test
    void bridgeAllowedUnderTwoHighCeilingButPillarIsNot() {
        BridgePillarStrategy strategy = strategyWithBlocks();

        List<Edge> edges = strategy.findEdges(new Node(0, 1, 0), tunnelWorld());

        assertTrue(edges.stream().anyMatch(edge ->
            edge.getType() == BuiltinMovementType.BRIDGE
                && edge.getTarget().equals(new Node(1, 1, 0))));
        assertFalse(edges.stream().anyMatch(edge -> edge.getType() == BuiltinMovementType.PILLAR));
    }

    private static BridgePillarStrategy strategyWithBlocks() {
        MovementSync movementSync = new MovementSync();
        ItemStack[] inventory = new ItemStack[45];
        inventory[36] = new ItemStack(ItemRegistry.Instance.getItem("stone").getId(), 64);
        movementSync.getInventoryManager().setContainerItems(0, inventory);
        return new BridgePillarStrategy();
    }

    private static World carpetWorld() {
        return new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                return x == 0 && y == 0 && z == 0 ? WHITE_CARPET : AIR;
            }
        };
    }

    /** Floor at y=0 only for x<=0 (a gap ahead), ceiling at y=3: two blocks of head room. */
    private static World tunnelWorld() {
        return new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                return y == 3 || y == 0 && x <= 0 ? STONE : AIR;
            }
        };
    }
}
