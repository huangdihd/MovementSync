package xin.bbtt.pathfinding;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

final class JumpStrategyExactHeightTest {
    private static final int AIR = 0;
    private static final int STONE = 1;
    private static final int WHITE_CARPET = 12694;
    private static final int BOTTOM_STONE_SLAB = 13197;
    private static final int TOP_OAK_SLAB = 13129;

    @Test
    void rejectsIntegerAdjacentNodeWhoseExactRiseExceedsJumpHeight() {
        World world = worldWithSupports(WHITE_CARPET, STONE);

        List<Edge> edges = new JumpStrategy(false).findEdges(new Node(0, 1, 0), world);

        assertFalse(edges.stream().anyMatch(edge -> edge.getTarget().equals(new Node(1, 2, 0))),
            "carpet top 0.0625 to raised stone top 2.0 is not a reachable jump");
    }

    @Test
    void keepsOneBlockExactJump() {
        World world = worldWithSupports(STONE, STONE);

        List<Edge> edges = new JumpStrategy(false).findEdges(new Node(0, 1, 0), world);

        assertTrue(edges.stream().anyMatch(edge -> edge.getTarget().equals(new Node(1, 2, 0))));
    }

    @Test
    void walkRejectsSameNodeRiseAboveStepHeight() {
        World world = worldWithSupportsAtLevels(WHITE_CARPET, 0, TOP_OAK_SLAB, 0);

        List<Edge> edges = new WalkStrategy(false).findEdges(new Node(0, 1, 0), world);

        assertFalse(edges.stream().anyMatch(edge -> edge.getTarget().equals(new Node(1, 1, 0))),
            "carpet top 0.0625 to top-slab top 1.0 exceeds the 0.6 step limit");
    }

    @Test
    void jumpFindsSameNodeYRiseAboveStepHeight() {
        World world = worldWithSupportsAtLevels(WHITE_CARPET, 0, STONE, 0);

        List<Edge> edges = new JumpStrategy(false).findEdges(new Node(0, 1, 0), world);

        assertTrue(edges.stream().anyMatch(edge ->
            edge.getType() == BuiltinMovementType.JUMP
                && edge.getTarget().equals(new Node(1, 1, 0))),
            "carpet top 0.0625 to full-block top 1.0 requires a jump within the same nodeY");
    }

    @Test
    void sameNodeYJumpAllowsDestinationCeilingWhenHeadBumpStillLands() {
        World world = new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (z != 0) return AIR;
                if (x == 0 && y == 0) return WHITE_CARPET;
                if (x == 1 && y == 0) return STONE;
                if (x == 1 && y == 3) return STONE;
                return AIR;
            }
        };
        assertTrue(StandablePositionResolver.feetY(world, 1, 1, 0).isPresent(),
            "the destination ceiling must still leave enough room to stand");

        List<Edge> edges = new JumpStrategy(false).findEdges(new Node(0, 1, 0), world);

        assertTrue(edges.stream().anyMatch(edge ->
            edge.getType() == BuiltinMovementType.JUMP
                && edge.getTarget().equals(new Node(1, 1, 0))),
            "head contact may truncate upward velocity while still leaving enough height to land");
    }

    @Test
    void sameNodeYJumpRejectsHeadBumpBeforeReachingLandingHeight() {
        World world = new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (z != 0) return AIR;
                if (x == 0 && y == 0) return WHITE_CARPET;
                if (x == 0 && y == 2) return STONE;
                if (x == 1 && y == 0) return STONE;
                return AIR;
            }
        };

        List<Edge> edges = new JumpStrategy(false).findEdges(new Node(0, 1, 0), world);

        assertFalse(edges.stream().anyMatch(edge ->
            edge.getType() == BuiltinMovementType.JUMP
                && edge.getTarget().equals(new Node(1, 1, 0))),
            "a head bump below the required landing height must remain unreachable");
    }

    @Test
    void raisedCarpetJumpCanLandByFootprintBeforeCentering() {
        World world = new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (z != 0) return AIR;
                if (x == 0 && y == 0) return WHITE_CARPET;
                if (x == 1 && y == 1) return WHITE_CARPET;
                if (x == 1 && y == 3) return STONE;
                return AIR;
            }
        };

        List<Edge> edges = new JumpStrategy(false).findEdges(new Node(0, 1, 0), world);

        assertTrue(edges.stream().anyMatch(edge ->
            edge.getType() == BuiltinMovementType.JUMP
                && edge.getTarget().equals(new Node(1, 2, 0))),
            "the player footprint may catch the target support before its center arrives");
    }

    @Test
    void bottomSlabJumpCanLandOnRaisedCarpetByFootprint() {
        World world = new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (z != 0) return AIR;
                if (x == 0 && y == 0) return BOTTOM_STONE_SLAB;
                if (x == 1 && y == 1) return WHITE_CARPET;
                if (x == 1 && y == 3) return STONE;
                return AIR;
            }
        };
        assertEquals(0.5, StandablePositionResolver.feetY(world, 0, 1, 0).orElseThrow(), 1.0e-9);
        assertEquals(1.0625, StandablePositionResolver.feetY(world, 1, 2, 0).orElseThrow(), 1.0e-9);

        List<Edge> edges = new JumpStrategy(false).findEdges(new Node(0, 1, 0), world);

        assertTrue(edges.stream().anyMatch(edge ->
            edge.getType() == BuiltinMovementType.JUMP
                && edge.getTarget().equals(new Node(1, 2, 0))));
    }

    @Test
    void twoBlockGapUsesSprintJumpTrajectory() {
        World world = new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (z != 0 || y != 0) return AIR;
                return x == 0 || x == 3 ? STONE : AIR;
            }
        };

        List<Edge> edges = new GapJumpStrategy().findEdges(new Node(0, 1, 0), world);

        assertTrue(edges.stream().anyMatch(edge ->
            edge.getType() == BuiltinMovementType.GAP_JUMP
                && edge.getTarget().equals(new Node(3, 1, 0))),
            "two-block gaps require the same 1.3x sprint speed used during execution");
    }

    @Test
    void fallUsesExactDropRatherThanIntegerNodeDifference() {
        World world = worldWithSupportsAtLevels(STONE, 0, WHITE_CARPET, -1);

        List<Edge> edges = new FallStrategy(1).findEdges(new Node(0, 1, 0), world);

        assertFalse(edges.stream().anyMatch(edge -> edge.getTarget().equals(new Node(1, 0, 0))),
            "stone top 1.0 to carpet top -0.9375 exceeds a one-block fall limit");
    }

    @Test
    void walkDigFallbackKeepsJumpReachableSupportAboveStepHeight() {
        World world = worldWithObstacle(WHITE_CARPET, 0, STONE, 0, 1);

        List<Edge> edges = new WalkStrategy(true).findEdges(new Node(0, 1, 0), world);

        assertTrue(edges.stream().anyMatch(edge ->
            edge.getType() == BuiltinMovementType.DIG
                && edge.getTarget().equals(new Node(1, 1, 0))));
    }

    @Test
    void flatDigDoesNotRequireJumpClearanceInATwoBlockHighTunnel() {
        World world = new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (z != 0) return AIR;
                if (y == 0) return STONE;
                if (x == 0 && y == 3) return STONE; // ceiling above two air blocks
                if (x == 1 && y == 1) return STONE; // wall to dig at foot level
                return AIR;
            }
        };
        Node source = new Node(0, 1, 0);
        assertTrue(StandablePositionResolver.feetY(world, source.x, source.y, source.z).isPresent(),
            "the regression fixture must begin at a physically standable position");

        List<Edge> edges = new WalkStrategy(true).findEdges(source, world);

        assertTrue(edges.stream().anyMatch(edge ->
            edge.getType() == BuiltinMovementType.DIG
                && edge.getTarget().equals(new Node(1, 1, 0))),
            "level digging should require walking clearance, not jump clearance");
    }

    @Test
    void walkDigFallbackRejectsJumpWhenSourceOverheadIsBlocked() {
        World world = new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (z != 0) return AIR;
                if (x == 0 && y == 0) return WHITE_CARPET;
                if (x == 0 && y == 2) return STONE;
                if (x == 1 && (y == 0 || y == 1)) return STONE;
                return AIR;
            }
        };

        List<Edge> edges = new WalkStrategy(true).findEdges(new Node(0, 1, 0), world);

        assertFalse(edges.stream().anyMatch(edge ->
            edge.getType() == BuiltinMovementType.DIG
                && edge.getTarget().equals(new Node(1, 1, 0))));
    }

    @Test
    void jumpDigFallbackRejectsSupportAboveJumpHeight() {
        World world = worldWithObstacle(WHITE_CARPET, 0, STONE, 1, 2);

        List<Edge> edges = new JumpStrategy(true).findEdges(new Node(0, 1, 0), world);

        assertFalse(edges.stream().anyMatch(edge ->
            edge.getType() == BuiltinMovementType.DIG
                && edge.getTarget().equals(new Node(1, 2, 0))));
    }

    private static World worldWithSupports(int sourceState, int targetState) {
        return worldWithSupportsAtLevels(sourceState, 0, targetState, 1);
    }

    private static World worldWithSupportsAtLevels(
            int sourceState, int sourceY, int targetState, int targetY) {
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
                if (z != 0) return AIR;
                if (x == 0 && y == sourceY) return sourceState;
                if (x == 1 && y == targetY) return targetState;
                return AIR;
            }
        };
    }

    private static World worldWithObstacle(
            int sourceState, int sourceY, int supportState, int supportY, int obstacleY) {
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
                if (z != 0) return AIR;
                if (x == 0 && y == sourceY) return sourceState;
                if (x == 1 && y == supportY) return supportState;
                if (x == 1 && y == obstacleY) return STONE;
                return AIR;
            }
        };
    }
}
