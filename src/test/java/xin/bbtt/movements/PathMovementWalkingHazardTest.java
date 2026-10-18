package xin.bbtt.movements;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.MovementSync;
import xin.bbtt.pathfinding.BuiltinMovementType;
import xin.bbtt.pathfinding.Node;
import xin.bbtt.pathfinding.PathStep;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

/** Real physics regression: the first tick is supported but its coast reaches lava. */
final class PathMovementWalkingHazardTest {
    private static final BlockState LAVA = new BlockState("minecraft:lava", 0, Map.of(), "empty", 0, false, "");
    private static final PlayerInput FORWARD = new PlayerInput(true, false, false, false, false, false, false);

    @AfterEach void resetSingleton() { MovementSync.INSTANCE = null; }

    private static World shore(boolean bridge) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d p) {
                return p.y < 1 && (p.x < 1 || p.x >= 3 || bridge) ? 1 : 0;
            }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (!bridge && p.y < 1 && p.x >= 1 && p.x < 3) return LAVA;
                return super.getBlockStateAt(p);
            }
        };
    }

    private static PhysicsState start() {
        return new PhysicsState(new Vector3d(.4, 1, .5), new Vector3d(.1, 0, 0), true);
    }

    private static PhysicsState step(World world, PhysicsState state, PlayerInput input, float yaw) {
        var result = new WalkingPhysics(input, yaw).step(world, state, PlayerPose.HALF_WIDTH, (double) 1.8f);
        return new PhysicsState(result.position(), result.velocity(), result.onGround(), result.fallDistance(), result.pose());
    }

    @Test void supportedFirstTickIsNotAnAbortProof() {
        World world = shore(false);
        PhysicsState first = step(world, start(), FORWARD, -90);
        assertTrue(first.onGround());
        assertTrue(FlightHazards.safeLanding(world, first.position()));
        boolean touchedLava = false;
        PhysicsState state = first;
        for (int tick = 0; tick < 40; tick++) {
            state = step(world, state, PlayerInput.NONE, -90);
            touchedLava |= !FlightHazards.safeLanding(world, state.position());
        }
        assertTrue(touchedLava, "unprotected command must reproduce coast into lava");
        assertFalse(DeploymentSafety.assessWalking(world, start(), MovementParameters.DEFAULT, -90, FORWARD).safe());
        assertTrue(DeploymentSafety.assessWalking(world, start(), MovementParameters.DEFAULT, -90, PlayerInput.NONE).safe());
    }

    @Test void pathRejectsDepartureBeforeInputAndInterruptedCoastStaysSafe() throws Exception {
        World world = shore(false);
        MovementSync sync = new MovementSync();
        sync.onRecoveryTeleport();
        Field field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(sync, world);
        sync.position.set(start().position());
        sync.velocity.set(start().velocity());
        sync.onGround.set(true);
        PathMovement movement = new PathMovement(List.of(
                new PathStep(new Node(0, 1, 0), BuiltinMovementType.WALK),
                new PathStep(new Node(3, 1, 0), BuiltinMovementType.WALK)));
        movement.init();
        movement.onTick();
        assertEquals(PlayerInput.NONE, sync.getInput(), "unsafe command must never be issued");
        PhysicsState state = step(world, start(), sync.getInput(), sync.yaw.get());
        movement.onPause();
        for (int tick = 0; tick < 80; tick++) {
            assertTrue(FlightHazards.safeLanding(world, state.position()));
            assertTrue(state.onGround());
            state = step(world, state, sync.getInput(), sync.yaw.get());
        }
        assertEquals(0, state.velocity().x);
    }

    @Test void supportedBridgeAndActualYawRemainAdmissible() {
        assertTrue(DeploymentSafety.assessWalking(shore(true), start(), MovementParameters.DEFAULT, -90, FORWARD).safe());
        // Facing away from the lava is safe; the guard must use the issued yaw.
        assertTrue(DeploymentSafety.assessWalking(shore(false), start(), MovementParameters.DEFAULT, 90, FORWARD).safe());
    }
}
