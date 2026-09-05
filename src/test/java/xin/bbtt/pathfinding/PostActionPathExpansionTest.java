package xin.bbtt.pathfinding;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import java.util.List;
import java.util.Map;
import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.MovementSync;
import xin.bbtt.inventory.ItemRegistry;
import xin.bbtt.world.World;

final class PostActionPathExpansionTest {
    private static final BlockState STONE = new BlockState(
        "minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState(
        "minecraft:air", 0, Map.of(), "empty", 0.0, false, "AIR");

    @AfterEach
    void resetSingleton() {
        MovementSync.INSTANCE = null;
    }

    @Test
    void plannerContinuesPastADigNodeUsingTheExpectedClearedCell() {
        World world = new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public BlockState getBlockStateAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (y == 63) return STONE;
                if (x == 1 && z == 0 && (y == 64 || y == 65)) return STONE;
                return AIR;
            }
        };
        Node goal = new Node(2, 64, 0);
        PathfindingContext context = corridor(
            new PathfindingContextBuilder(world).addWalk(true).build(), 0, 2, 64, 64);

        List<PathStep> path = new DStarLite(
            new Node(0, 64, 0), goal, context
        ).findPath(2000);

        assertReaches(path, goal, "planner must continue from the cell that DIG will clear");
    }

    @Test
    void plannerCanJumpImmediatelyAfterDigClearsFeetAndHeadCells() {
        World world = blockIdWorld((x, y, z) -> {
            if (z != 0) return 0;
            if ((x == 0 || x == 1) && y == 63) return 1;
            if (x == 1 && (y == 64 || y == 65)) return 1;
            if (x == 2 && y == 64) return 1;
            return 0;
        });
        Node goal = new Node(2, 65, 0);
        PathfindingContext context = corridor(
            new PathfindingContextBuilder(world).addWalk(true).addJump(false).build(),
            0, 2, 64, 65);

        List<PathStep> path = new DStarLite(new Node(0, 64, 0), goal, context)
            .findPath(2000);

        assertReaches(path, goal, "DIG must remove both stale source collisions before JUMP expansion");
        assertEquals(BuiltinMovementType.DIG, path.get(1).getType());
        assertEquals(BuiltinMovementType.JUMP, path.get(2).getType());
    }

    @Test
    void plannerContinuesPastABridgeNodeUsingTheExpectedPlacedSupport() {
        World world = blockIdWorld((x, y, z) -> y == 63 && (x == 0 || x >= 2) ? 1 : 0);
        installInventoryWithBlocks();
        Node goal = new Node(2, 64, 0);
        PathfindingContext context = corridor(
            new PathfindingContextBuilder(world).addWalk(false).addBridgePillar().build(),
            0, 2, 64, 64);

        List<PathStep> path = new DStarLite(
            new Node(0, 64, 0), goal, context
        ).findPath(2000);

        assertReaches(path, goal, "planner must continue from the support that BRIDGE will place");
    }

    @Test
    void bridgeDoesNotEnterAnUnloadedNeighboringChunk() {
        World world = new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return chunkX == 0 && chunkZ == 0;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                return x == 15 && y == 63 && z == 0 ? 1 : 0;
            }
        };
        installInventoryWithBlocks();

        List<Edge> edges = new BridgePillarStrategy().findEdges(new Node(15, 64, 0), world);

        assertFalse(edges.stream().anyMatch(edge ->
            edge.getType() == BuiltinMovementType.BRIDGE
                && edge.getTarget().equals(new Node(16, 64, 0))),
            "placement must fail closed at a loaded-to-unloaded chunk boundary");
        assertFalse(StandablePositionResolver.canStepToFeetY(
                world, new Node(16, 64, 0), 64.0),
            "an unloaded source must not synthesize post-placement support");
    }

    @Test
    void plannerContinuesPastAPillarNodeUsingTheExpectedPlacedSupport() {
        World world = blockIdWorld((x, y, z) -> y == 63 ? 1 : 0);
        installInventoryWithBlocks();
        Node goal = new Node(0, 66, 0);

        PathfindingContext context = new PathfindingContextBuilder(world)
            .addBridgePillar()
            .build();
        List<PathStep> path = new DStarLite(
            new Node(0, 64, 0), goal, context
        ).findPath(2000);

        assertReaches(path, goal, "planner must continue from the support that PILLAR will place");
    }

    private static void assertReaches(List<PathStep> path, Node goal, String message) {
        assertEquals(goal, path.isEmpty() ? null : path.get(path.size() - 1).getNode(),
            message + ": " + path);
    }

    private static PathfindingContext corridor(
            PathfindingContext delegate, int minX, int maxX, int minY, int maxY) {
        return new PathfindingContext() {
            @Override
            public List<Edge> getEdges(Node node) {
                return delegate.getEdges(node).stream()
                    .filter(edge -> edge.getTarget().z == 0)
                    .filter(edge -> edge.getTarget().x >= minX && edge.getTarget().x <= maxX)
                    .filter(edge -> edge.getTarget().y >= minY && edge.getTarget().y <= maxY)
                    .toList();
            }

            @Override
            public double getHeuristic(Node a, Node b) {
                return delegate.getHeuristic(a, b);
            }
        };
    }

    private static void installInventoryWithBlocks() {
        MovementSync movementSync = new MovementSync();
        ItemStack[] inventory = new ItemStack[45];
        inventory[36] = new ItemStack(ItemRegistry.Instance.getItem("stone").getId(), 64);
        movementSync.getInventoryManager().setContainerItems(0, inventory);
    }

    private static World blockIdWorld(BlockLookup lookup) {
        return new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                return lookup.get(
                    (int) Math.floor(position.x),
                    (int) Math.floor(position.y),
                    (int) Math.floor(position.z)
                );
            }
        };
    }

    @FunctionalInterface
    private interface BlockLookup {
        int get(int x, int y, int z);
    }
}
