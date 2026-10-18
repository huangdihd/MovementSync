package xin.bbtt.movements;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.MovementSync;
import xin.bbtt.pathfinding.*;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

/** Reported round-13 coordinates; body walls are explicit hypotheses, not captured geometry. */
final class PathMovementPhysicalStallTest {
    private static final Node FROM = new Node(178, 39, -22);
    private static final Node TO = new Node(179, 39, -23);
    private static final BlockState ROCK = new BlockState("minecraft:netherrack", 1, Map.of(), "block", .4, true, "STONE");
    private static final BlockState SOIL = new BlockState("minecraft:soul_soil", 1, Map.of(), "block", .4, true, "SOIL");
    private static final BlockState AIR = new BlockState("minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");

    @AfterEach void reset() { MovementSync.INSTANCE = null; }

    private static World world(boolean alternate) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                int x = (int) Math.floor(p.x), y = (int) Math.floor(p.y), z = (int) Math.floor(p.z);
                if (y <= 38) return x == 179 && y == 38 && z == -23 ? SOIL : ROCK;
                if (y >= 41) return ROCK; // no jumping over the hypothetical corner
                boolean bodyWall = alternate
                        ? x == 179 && z == -22 || x == 178 && z == -23
                        : !(x == 178 && z == -22 || x == 179 && z == -23);
                return bodyWall ? ROCK : AIR;
            }
        };
    }

    private static MovementSync ready(World world) throws Exception {
        MovementSync sync = new MovementSync();
        sync.onRecoveryTeleport();
        Field field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(sync, world);
        // The reported decimal coordinates are rounded: vanilla's float-derived
        // half-width exceeds .3, so literal 178.7 already penetrates the east
        // wall. Start just inside the corner instead of embedding the body in it.
        sync.position.set(new Vector3d(179 - PlayerPose.HALF_WIDTH - 1.0e-6, 39,
                -22 + PlayerPose.HALF_WIDTH + 1.0e-6));
        Vector3d position = sync.position.get();
        assertFalse(world.isBoxColliding(position.x - PlayerPose.HALF_WIDTH, position.y,
                position.z - PlayerPose.HALF_WIDTH, position.x + PlayerPose.HALF_WIDTH,
                position.y + 1.8, position.z + PlayerPose.HALF_WIDTH),
                "stall fixture must start with a non-penetrating player body");
        sync.velocity.set(new Vector3d(0, -.0784, 0));
        sync.onGround.set(true);
        sync.setActiveGoal(new Vector3i(TO.x, TO.y, TO.z));
        return sync;
    }

    private static void physicalTick(MovementSync sync) {
        var result = new WalkingPhysics(sync.getInput(), sync.yaw.get(), true, sync.getMovementParameters())
                .step(sync.getWorld(), new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get()),
                        PlayerPose.HALF_WIDTH, 1.8);
        sync.position.set(result.position());
        sync.velocity.set(result.velocity());
        sync.onGround.set(result.onGround());
    }

    @SuppressWarnings("unchecked")
    private static Set<ExcludingPathfindingContext.Transition> excluded(PathMovement movement) throws Exception {
        Field field = PathMovement.class.getDeclaredField("rejectedTransitions");
        field.setAccessible(true);
        return (Set<ExcludingPathfindingContext.Transition>) field.get(movement);
    }

    @Test void blockedDiagonalExcludesDirectedEdgeBeforeGlobalWatchdog() throws Exception {
        MovementSync sync = ready(world(false));
        PathMovement movement = new PathMovement(List.of(new PathStep(FROM, BuiltinMovementType.WALK),
                new PathStep(TO, BuiltinMovementType.WALK)));
        movement.init();
        for (int tick = 0; tick < 20 && !movement.isFinished(); tick++) {
            movement.onTick();
            physicalTick(sync);
        }
        assertTrue(excluded(movement).contains(new ExcludingPathfindingContext.Transition(FROM, TO)));
        assertEquals(PathMovement.Completion.FAILED, movement.completion());
        assertEquals(PlayerInput.NONE, sync.getInput());
    }

    @Test void blockedDiagonalFullReplanFindsAndExecutesAlternateRoute() throws Exception {
        MovementSync sync = ready(world(true));
        PathMovement movement = new PathMovement(List.of(new PathStep(FROM, BuiltinMovementType.WALK),
                new PathStep(TO, BuiltinMovementType.WALK)));
        movement.init();
        for (int tick = 0; tick < 160 && !movement.isFinished(); tick++) {
            movement.onTick();
            physicalTick(sync);
        }
        assertTrue(excluded(movement).contains(new ExcludingPathfindingContext.Transition(FROM, TO)));
        assertTrue(movement.isFinished());
        assertEquals(PathMovement.Completion.NORMAL, movement.completion());
        assertTrue(sync.position.get().distance(new Vector3d(179.5, 39, -22.5)) < .5);
    }
}
