package xin.bbtt.movements;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Map;
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

/** Round-nine departure: real collision/abort physics, not a mocked safety verdict. */
final class PathMovementRejectedEdgeTest {
    private static final Node START = new Node(27, 33, -1);
    private static final Node GOAL = new Node(30, 33, -1);
    private static final BlockState ROCK = new BlockState("minecraft:netherrack", 1, Map.of(), "block", .4, true, "STONE");
    private static final BlockState AIR = new BlockState("minecraft:cave_air", 0, Map.of(), "empty", 0, false, "AIR");
    private static final BlockState LAVA = new BlockState("minecraft:lava", 0, Map.of(), "empty", 0, false, "LAVA");

    @AfterEach void reset() { MovementSync.INSTANCE = null; }

    private static World world(boolean alternate) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                int x = (int) Math.floor(p.x), y = (int) Math.floor(p.y), z = (int) Math.floor(p.z);
                boolean support = z == -1 && (x == 27 || x == 30)
                        || alternate && z == -2 && x >= 27 && x <= 30;
                if (y == 32 && support) return ROCK;
                if (y <= 31) return LAVA;
                return AIR;
            }
        };
    }

    private static PhysicsState initial() {
        return new PhysicsState(new Vector3d(27.41, 33, -.504), new Vector3d(0, -.0784, 0), true);
    }

    private static MovementSync ready(World world) throws Exception {
        MovementSync sync = new MovementSync();
        sync.onRecoveryTeleport();
        Field field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(sync, world);
        sync.position.set(initial().position());
        sync.velocity.set(initial().velocity());
        sync.onGround.set(true);
        sync.setActiveGoal(new Vector3i(GOAL.x, GOAL.y, GOAL.z));
        return sync;
    }

    private static PathMovement movement() {
        PathMovement movement = new PathMovement(List.of(
                new PathStep(START, BuiltinMovementType.WALK),
                new PathStep(GOAL, BuiltinMovementType.GAP_JUMP)));
        movement.init();
        return movement;
    }

    @Test void rejectedNumericGapFailsImmediatelyAndNeutralBranchStaysSupported() throws Exception {
        World world = world(false);
        MovementSync sync = ready(world);
        var command = JumpInputController.toward(initial().position(), new Vector3d(30.5, 33, -.5),
                initial().velocity(), true, true);
        assertFalse(DeploymentSafety.assessWalking(world, initial(), MovementParameters.DEFAULT,
                command.yaw(), command.input()).safe());
        assertTrue(DeploymentSafety.assessWalking(world, initial(), MovementParameters.DEFAULT,
                command.yaw(), PlayerInput.NONE).safe());
        PathMovement movement = movement();
        movement.onTick();
        assertEquals(PlayerInput.NONE, sync.getInput());
        assertTrue(movement.isFinished(), "unsafe edge must not wait for the 200-tick watchdog");
        assertEquals(PathMovement.Completion.FAILED, movement.completion());
        assertEquals(PathMovement.FailureReason.UNSAFE_ROUTE, movement.failureReason());
        assertNull(sync.getActiveGoal());
        PhysicsState state = initial();
        for (int tick = 0; tick < 80; tick++) {
            var result = new WalkingPhysics(sync.getInput(), sync.yaw.get()).step(world, state,
                    PlayerPose.HALF_WIDTH, (double) 1.8f);
            state = new PhysicsState(result.position(), result.velocity(), result.onGround(), result.fallDistance(), result.pose());
            assertTrue(state.onGround());
            assertTrue(FlightHazards.safeLanding(world, state.position()));
        }
        assertEquals(27.41, state.position().x, 1e-9);
        assertEquals(33, state.position().y, 1e-9);
    }

    @Test void rejectedGapReplansToAnAlternateRouteInTheSameTick() throws Exception {
        MovementSync sync = ready(world(true));
        PathMovement movement = movement();
        movement.onTick();
        assertFalse(movement.isFinished());
        assertEquals(PlayerInput.NONE, sync.getInput());
        Field field = PathMovement.class.getDeclaredField("path");
        field.setAccessible(true);
        @SuppressWarnings("unchecked") List<PathStep> route = (List<PathStep>) field.get(movement);
        assertTrue(route.size() > 2);
        assertEquals(START, route.get(0).getNode());
        assertNotEquals(GOAL, route.get(1).getNode(), "must not repeatedly select the rejected directed edge");
        assertEquals(GOAL, route.get(route.size() - 1).getNode());
        assertNull(movement.failureReason());
    }

    @Test void stoppedPathClearsOwnedKeysAndPendingJumpAndCannotResume() throws Exception {
        MovementSync sync = ready(world(true));
        PathMovement movement = movement();
        PlayerInput owned = new PlayerInput(true, false, false, false, true, false, false);
        Field field = PathMovement.class.getDeclaredField("issuedInput");
        field.setAccessible(true);
        field.set(movement, owned);
        sync.setInput(owned);
        sync.jump();
        movement.onStop();
        assertEquals(PlayerInput.NONE, sync.snapshotInput());
        movement.requestRepath();
        movement.onTick();
        assertEquals(PlayerInput.NONE, sync.snapshotInput());
        assertNull(movement.failureReason());
    }

    @Test void stoppedPathDoesNotClearAnotherOwnersKeys() throws Exception {
        MovementSync sync = ready(world(true));
        PathMovement movement = movement();
        PlayerInput owned = new PlayerInput(true, false, false, false, false, false, false);
        Field field = PathMovement.class.getDeclaredField("issuedInput");
        field.setAccessible(true);
        field.set(movement, owned);
        PlayerInput replacement = new PlayerInput(false, true, false, false, false, false, false);
        sync.setInput(replacement);
        movement.onStop();
        movement.onTick();
        assertSame(replacement, sync.getInput());
    }

    @Test void failedPathCannotIssueCommandsOnLaterTicks() throws Exception {
        MovementSync sync = ready(world(false));
        PathMovement movement = movement();
        movement.onTick();
        assertEquals(PathMovement.Completion.FAILED, movement.completion());
        PlayerInput replacement = new PlayerInput(false, true, false, false, false, false, false);
        sync.setInput(replacement);
        movement.requestRepath();
        for (int tick = 0; tick < 40; tick++) movement.onTick();
        assertSame(replacement, sync.getInput());
    }

}
