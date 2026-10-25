package xin.bbtt.pathfinding;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import xin.bbtt.Block.BlockStateParser;
import xin.bbtt.world.World;

final class EnvironmentalTraversalTest {
    @Test
    void waterSurfaceNeedsNeitherSolidFloorNorPlacement() {
        World world = waterWorld(86);
        var position = EnvironmentalTraversal.position(world, new Node(0, 1, 0));
        assertTrue(position.isPresent());
        assertEquals(1 - 1.0 / 9 - 0.25, position.orElseThrow().y, 1.0e-9);
        assertTrue(StandablePositionResolver.feetY(world, 0, 1, 0).isEmpty());
        assertTrue(hasEdge(new EnvironmentalStrategy().findEdges(new Node(0, 1, 0), world),
                new Node(1, 1, 0), BuiltinMovementType.SWIM));
    }

    @Test
    void rejectsLavaAndSubmergedNodes() {
        assertTrue(EnvironmentalTraversal.position(waterWorld(102), new Node(0, 1, 0)).isEmpty());
        assertTrue(EnvironmentalTraversal.position(waterWorld(86), new Node(0, 0, 0)).isEmpty());
        assertTrue(new EnvironmentalStrategy().findEdges(new Node(0, 1, 0), waterWorld(102)).isEmpty());
    }

    @ParameterizedTest
    @ValueSource(strings = {"kelp", "kelp_plant", "seagrass", "tall_seagrass"})
    void aquaticPlantsAllowVerticalRecoveryToOpenWater(String name) {
        World world = waterWorld(stateId(name));
        Node submerged = new Node(0, -3, 0);
        Vector3d center = new Vector3d(0.5, -3, 0.5);
        assertTrue(EnvironmentalTraversal.inWater(world, center));
        assertTrue(EnvironmentalTraversal.position(world, submerged).isEmpty());
        assertAll(
                () -> assertEquals(center, EnvironmentalTraversal.recoveryPosition(world, submerged).orElseThrow()),
                () -> assertEquals(center, EnvironmentalTraversal.destination(world, submerged).orElseThrow()),
                () -> assertEquals(1 - 1.0 / 9 - 0.25,
                        EnvironmentalTraversal.position(world, new Node(0, 1, 0)).orElseThrow().y, 1.0e-9));
    }

    @ParameterizedTest
    @ValueSource(strings = {"kelp", "kelp_plant", "seagrass", "tall_seagrass"})
    void aquaticPlantRecoveryOffersOnlyUpwardSwimEdge(String name) {
        World world = waterWorld(stateId(name));
        List<Edge> edges = new EnvironmentalStrategy().findEdges(new Node(0, -3, 0), world);
        assertEquals(1, edges.size());
        assertTrue(hasEdge(edges, new Node(0, -2, 0), BuiltinMovementType.SWIM));
    }

    @ParameterizedTest
    @ValueSource(strings = {"kelp", "kelp_plant", "seagrass", "tall_seagrass"})
    void ceilingAboveAquaticPlantsBlocksRecovery(String name) {
        int plant = stateId(name);
        int stone = stateId("stone");
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) {
                if (position.y >= 1 && position.y < 2) return stone;
                return position.y < 1 && position.y >= -5 ? plant : 0;
            }
        };
        Node submerged = new Node(0, -3, 0);
        assertTrue(EnvironmentalTraversal.inWater(world, new Vector3d(0.5, -3, 0.5)));
        assertTrue(EnvironmentalTraversal.recoveryPosition(world, submerged).isEmpty());
        assertTrue(EnvironmentalTraversal.position(world, new Node(0, 1, 0)).isEmpty());
        assertTrue(new EnvironmentalStrategy().findEdges(submerged, world).isEmpty());
    }

    @Test
    void waterloggedSolidCannotBecomeRecoveryOrSurfaceDestination() {
        World world = waterWorld(stateId("oak_slab", "waterlogged", "true", "type", "top"));
        Node submerged = new Node(0, -3, 0);
        assertTrue(EnvironmentalTraversal.inWater(world, new Vector3d(0.5, -3, 0.5)));
        assertTrue(EnvironmentalTraversal.recoveryPosition(world, submerged).isEmpty());
        assertTrue(EnvironmentalTraversal.position(world, new Node(0, 1, 0)).isEmpty());
        assertTrue(new EnvironmentalStrategy().findEdges(submerged, world).isEmpty());
    }

    @Test
    void ladderAndVineSupportBothVerticalDirections() {
        for (int block : new int[]{5519, 8188}) {
            World world = columnWorld(block, false);
            List<Edge> edges = new EnvironmentalStrategy().findEdges(new Node(0, 2, 0), world);
            assertTrue(hasEdge(edges, new Node(0, 3, 0), BuiltinMovementType.CLIMB), "ascent " + block);
            assertTrue(hasEdge(edges, new Node(0, 1, 0), BuiltinMovementType.CLIMB), "descent " + block);
        }
    }

    @Test
    void ceilingBlocksClimbingAndUnloadedWaterIsNotRoutable() {
        World world = columnWorld(5519, true);
        assertFalse(hasEdge(new EnvironmentalStrategy().findEdges(new Node(0, 2, 0), world),
                new Node(0, 3, 0), BuiltinMovementType.CLIMB));
        World unloaded = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return false; }
            @Override public int getBlockAt(Vector3d position) { return 86; }
        };
        assertTrue(EnvironmentalTraversal.position(unloaded, new Node(0, 1, 0)).isEmpty());
    }

    @Test
    void verticalCommandsDoNotRequireHorizontalDistanceAndNeverSprint() {
        World ladder = columnWorld(5519, false);
        Vector3d position = new Vector3d(0.5, 2, 0.5);
        var up = EnvironmentalTraversal.toward(ladder, position, new Vector3d(position).add(0, 1, 0), new Vector3d(), false);
        assertTrue(up.input().jump());
        assertFalse(up.input().forward());
        assertFalse(up.input().sprint());
        var down = EnvironmentalTraversal.toward(ladder, position, new Vector3d(position).add(0, -1, 0), new Vector3d(), false);
        assertFalse(down.input().jump());
        assertFalse(down.input().sneak());
        var swim = EnvironmentalTraversal.toward(waterWorld(86), new Vector3d(0.5, 0.5, 0.5),
                new Vector3d(1.5, 0.64, 0.5), new Vector3d(), false);
        assertTrue(swim.input().jump());
        assertFalse(swim.input().sprint());
    }

    @Test
    void ladderConnectsGroundAndUpperLanding() {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                if (y == 0 || x == 1 && y <= 3) return 1;
                return x == 0 && y >= 1 && y <= 4 ? 5519 : 0;
            }
        };
        assertTrue(hasEdge(new EnvironmentalStrategy().findEdges(new Node(-1, 1, 0), world),
                new Node(0, 1, 0), BuiltinMovementType.CLIMB));
        assertTrue(hasEdge(new EnvironmentalStrategy().findEdges(new Node(0, 4, 0), world),
                new Node(1, 4, 0), BuiltinMovementType.CLIMB));
        assertTrue(hasEdge(new EnvironmentalStrategy().findEdges(new Node(1, 4, 0), world),
                new Node(0, 4, 0), BuiltinMovementType.CLIMB));
    }

    @Test
    void waterRouteConnectsBothShoresWithoutPlacement() {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) {
                if (position.y >= 1) return 0;
                return position.x >= 1 && position.x < 4 ? 86 : 1;
            }
        };
        var context = new PathfindingContextBuilder(world).addNoPlacementStrategies(false).build();
        Node goal = new Node(4, 1, 0);
        List<PathStep> path = new DStarLite(new Node(0, 1, 0), goal, context).findPath(200);
        assertFalse(path.isEmpty());
        assertEquals(goal, path.get(path.size() - 1).getNode());
        assertTrue(path.stream().anyMatch(step -> step.getType() == BuiltinMovementType.SWIM));
        assertFalse(path.stream().anyMatch(step -> step.getType() == BuiltinMovementType.BRIDGE));
    }

    private static int stateId(String name, String... properties) {
        for (int id = 0; id < BlockStateParser.getBlockStateRegistrySize(); id++) {
            var state = BlockStateParser.Instance.parseStateId(id);
            if (!state.blockName().equals(name)) continue;
            boolean matches = true;
            for (int index = 0; index < properties.length; index += 2) {
                matches &= properties[index + 1].equals(state.getProperty(properties[index]));
            }
            if (matches) return id;
        }
        throw new AssertionError("Unknown state " + name);
    }

    private static boolean hasEdge(List<Edge> edges, Node target, BuiltinMovementType type) {
        return edges.stream().anyMatch(edge -> edge.getTarget().equals(target) && edge.getType() == type);
    }

    private static World waterWorld(int block) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) {
                return position.y < 1 && position.y >= -5 ? block : 0;
            }
        };
    }

    private static World columnWorld(int block, boolean ceiling) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (y == 0 || ceiling && y == 4) return 1;
                return x == 0 && z == 0 && y >= 1 && y <= 5 ? block : 0;
            }
        };
    }
}
