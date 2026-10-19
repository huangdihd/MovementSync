package xin.bbtt.flight;

import java.util.List;
import java.util.Map;
import java.util.OptionalInt;
import java.util.UUID;
import java.util.concurrent.TimeUnit;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import xin.bbtt.Block.BlockState;
import xin.bbtt.Entity.Entity;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Observed positions/velocities, synthetic loaded lava sea: not a replay of unknown field terrain. */
class P5eLavaCruiseMarginTest extends ElytraCommandTestSupport {
    private static World sea() {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.y < 31) return new BlockState("lava", 1, Map.of(), "fluid", 0, false, "STONE");
                return super.getBlockStateAt(p);
            }
        };
    }

    @Test void attachedThrustMustClimbInsteadOfFollowingLowDescendingRoute() throws Exception {
        checkCruise(new Vector3d(494.08, 33.70, -2), new Vector3d(1.647, -.154, -.345), true);
    }

    @Test void unattachedFastLowCruiseStillRequestsEscape() throws Exception {
        checkCruise(new Vector3d(494.08, 33.70, -2), new Vector3d(1.647, -.154, -.345), false);
    }

    @Test void attachedThrustMustAlsoRescueObservedSlowState() throws Exception {
        checkCruise(new Vector3d(534.4, 32.68, -13.12), new Vector3d(.06229, -.07883, -.05688), true);
    }

    private void checkCruise(Vector3d position, Vector3d velocity, boolean attached) throws Exception {
        World world = sea();
        var worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
        movementSync.position.set(position);
        movementSync.velocity.set(velocity);
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        movementSync.entityId = 42;
        movementSync.yaw.set(-2621.8f);
        movementSync.pitch.set(5f);
        if (attached) {
            Entity rocket = new Entity(43, new UUID(0, 43), EntityType.FIREWORK_ROCKET,
                    position.x, position.y, position.z, 0, 0, 0, new Vector3d());
            rocket.getMetadata().put("attached_to_target", OptionalInt.of(42));
            world.getEntities().put(43, rocket);
        }
        assertEquals(attached, movementSync.hasAttachedFirework());
        Object owner = new Object();
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        var source = FlightControlAdmission.snapshot(movementSync);
        assertTrue(LocalGotoController.needsLavaMargin(world, source.state()));
        Vector3d goal = new Vector3d(778, 40, -31);
        var route = new WaypointFlightTarget(List.of(position,
                new Vector3d(position.x + 24, 32.39, position.z - 6), goal), goal, false);
        int[] inventory = new int[64];
        java.util.Arrays.fill(inventory, 1);
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route, goal, !attached,
                    () -> {}, null, new HazardRecovery.Context(inventory, 0, attached,
                            movementSync.getLandingMemory()), true, true));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertEquals(LocalGotoController.Status.SAFE_PREFIX, controller.latest().status());
            assertEquals(!attached, controller.latest().rocket(), "never stack an attached rocket");
            var controlsField = MovementSync.class.getDeclaredField("flightControls");
            controlsField.setAccessible(true);
            var pendingField = FlightControlAdmission.class.getDeclaredField("pending");
            pendingField.setAccessible(true);
            var envelope = (FlightControlAdmission.Envelope) pendingField.get(controlsField.get(movementSync));
            assertNotNull(envelope);
            var control = envelope.steps().get(0).control();
            var replay = LocalGotoController.replay(world, MovementParameters.DEFAULT, source.state(),
                    control, true, 12, goal, false);
            assertEquals(12, replay.steps().size());
            assertTrue(replay.end().position().y > position.y + 2,
                    () -> "low lava cruise must use thrust to climb: " + control + " end=" + replay.end());
            assertEquals(envelope.steps().size(), LocalGotoController.replay(world, MovementParameters.DEFAULT,
                    source.state(), control, false, envelope.steps().size(), goal, false).steps().size(),
                    "same controls must remain safe if attachment disappears");
        }
    }
}
