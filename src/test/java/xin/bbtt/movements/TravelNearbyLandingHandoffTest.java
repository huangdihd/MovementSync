package xin.bbtt.movements;

import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import xin.bbtt.Block.BlockState;
import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;
import xin.bbtt.pathfinding.*;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.WalkingPhysics;
import xin.bbtt.world.World;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

final class TravelNearbyLandingHandoffTest {
    private static final BlockState STONE = new BlockState(
            "minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState(
            "minecraft:air", 0, Map.of(), "empty", 0., false, "AIR");

    @AfterEach void reset() { MovementSync.INSTANCE = null; }

    private static MovementSync ready(World terrain, Vector3d position) {
        // Do not call this parameter "world": the inherited MovementSync.world
        // would shadow it inside the anonymous subclass and return an empty world.
        MovementSync sync = new MovementSync() {
            @Override public World getWorld() { return terrain; }
            @Override public boolean isGliding() { return false; }
        };
        sync.position.set(position);
        sync.velocity.set(new Vector3d());
        sync.onGround.set(true);
        return sync;
    }

    private static void physics(MovementSync sync) {
        var result = new WalkingPhysics(sync.snapshotInput(), sync.yaw.get()).step(sync.getWorld(),
                new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get()),
                (double) .6f / 2, 1.8);
        sync.position.set(result.position());
        sync.velocity.set(result.velocity());
        sync.onGround.set(result.onGround());
    }

    @ParameterizedTest
    @ValueSource(ints = {-3, -2, -1, 1, 2, 3})
    void nearbyLandingWalksRealStairRouteToSupportedGoal(int height) {
        // The endpoints are only two blocks apart; larger rises require a walking detour.
        int[][] columns = {{0, 0}, {0, 1}, {0, 2}, {1, 2}, {2, 2}, {2, 1}, {2, 0}};
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                for (int i = 0; i < columns.length; i++) {
                    int top = 64 + Integer.signum(height) * Math.min(i, Math.abs(height));
                    if (Math.floor(p.x) == columns[i][0] && Math.floor(p.z) == columns[i][1]
                            && p.y < top && p.y >= 58) return STONE;
                }
                return AIR;
            }
        };
        MovementSync sync = ready(world, new Vector3d(.5, 64, .5));
        Vector3i goal = new Vector3i(2, 64 + height, 0);
        long generation = sync.beginStaticNavigationRequest(goal);
        Movement flight = new Movement() {
            @Override public void init() {}
            @Override public void onTick() { setFinished(true); }
            @Override public long getTime() { return -1; }
            @Override public void onStop() {}
        };
        TravelMovement travel = new TravelMovement(goal, generation, true, new TravelMovement.Legs() {
            @Override public List<PathStep> plan(Vector3i destination) {
                Vector3d p = sync.position.get();
                return new DStarLite(new Node((int) Math.floor(p.x), StandablePositionResolver.nodeY(p.y),
                        (int) Math.floor(p.z)), new Node(goal.x, goal.y, goal.z), world).findPath(5000);
            }
            @Override public Movement walk(List<PathStep> path, Vector3i destination, boolean intermediate) {
                assertFalse(path.isEmpty());
                return new PathMovement(path, -1, false, destination, -1, generation);
            }
            @Override public Movement flight() { return flight; }
        });
        travel.init();
        for (int tick = 0; tick < 600 && !travel.isFinished(); tick++) {
            travel.onTick();
            // Never let residual physics hide an early completion.
            if (!travel.isFinished()) physics(sync);
        }
        assertTrue(travel.isFinished(), "position=" + sync.position.get());
        assertTrue(sync.isNavigationRequestCurrent(generation), "arrival must not abandon navigation");
        assertTrue(sync.onGround.get());
        assertTrue(Math.abs(sync.position.get().y - goal.y) < 1);
        assertTrue(Math.abs(sync.position.get().x - (goal.x + .5)) < .5);
        assertTrue(Math.abs(sync.position.get().z - (goal.z + .5)) < .5);
        assertNull(sync.getActiveGoal());
    }

    @Test void replanInGoalCellMustNotFinishBeforeTouchdown() {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) { return p.y < 60 ? STONE : AIR; }
        };
        MovementSync sync = ready(world, new Vector3d(175.05, 59.99, -31.5));
        Vector3i goal = new Vector3i(175, 60, -32);
        long generation = sync.beginStaticNavigationRequest(goal);
        sync.onGround.set(false);
        PathMovement path = new PathMovement(List.of(), -1, false, goal, -1, generation);
        path.init();
        path.onTick();
        assertFalse(path.isFinished(), "integer goal-cell equality cannot finish an airborne leg");
        assertEquals(goal, sync.getActiveGoal());
        sync.position.set(new Vector3d(175.5, 60, -31.5));
        sync.onGround.set(true);
        path.onTick();
        assertTrue(path.isFinished());
    }

    @Test void twentyBlocksBelowGoalCannotArrive() {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) { return p.y < 40 ? STONE : AIR; }
        };
        MovementSync sync = ready(world, new Vector3d(175.5, 40, -31.5));
        Vector3i goal = new Vector3i(175, 60, -32);
        long generation = sync.beginStaticNavigationRequest(goal);
        PathMovement path = new PathMovement(List.of(new PathStep(new Node(175, 40, -32),
                BuiltinMovementType.WALK)), -1, false, goal, -1, generation);
        path.onTick();
        assertFalse(path.isFinished());
        assertEquals(goal, sync.getActiveGoal());
    }
}
