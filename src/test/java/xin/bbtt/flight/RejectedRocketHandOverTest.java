package xin.bbtt.flight;

import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Low, slow glide over lava: a rejected rocket cannot keep the escape controls airborne
 * for a whole lifetime, but an observed rejection hands over to a proved unpowered reserve. */
class RejectedRocketHandOverTest extends ElytraCommandTestSupport {
    private static final double LAVA_TOP = 31;

    private static World sea() {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double x, double y, double z,
                    double X, double Y, double Z) { return Y > 128; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.y < LAVA_TOP) return new BlockState("lava", 1, Map.of(), "fluid", 0, false, "STONE");
                return super.getBlockStateAt(p);
            }
        };
    }

    private static PhysicsState fieldState() {
        return new PhysicsState(new Vector3d(376.4, 32.46, 12.7),
                new Vector3d(.379, -.081, -.013), false, 1, PlayerPose.GLIDING);
    }

    @Test void strictSearchFindsNoRocketWhoseRejectionStaysSafeForTheWholeLifetime() {
        var state = fieldState();
        var goal = new Vector3d(356.4, 32.46, 12.7);
        var escape = LocalGotoController.poweredEscape(sea(), MovementParameters.DEFAULT, state, 90, goal);
        assertFalse(escape.isEmpty());
        var episode = LocalGotoController.prepareRocketEpisode(sea(), MovementParameters.DEFAULT, state,
                escape.get(0).control(), escape.size(), FireworkSchedule.lifetimeBounds(1), goal,
                null, true, null);
        assertTrue(episode.prefix().isEmpty(),
                "fixture must need the hand-over: strict rejection proof is unavailable here");
    }

    @Test void observedRejectionHandsOverToProvedUnpoweredReserveAboveLava() throws Exception {
        var world = sea();
        var state = fieldState();
        var worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
        movementSync.position.set(state.position());
        movementSync.velocity.set(state.velocity());
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        Object owner = new Object();
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        var source = FlightControlAdmission.snapshot(movementSync);
        var goal = new Vector3d(356.4, 32.46, 12.7);
        var route = new WaypointFlightTarget(List.of(state.position(), goal), goal, false);
        int[] rockets = new int[64];
        java.util.Arrays.fill(rockets, 1);
        var resources = new HazardRecovery.Context(rockets, 0, false, movementSync.getLandingMemory());
        var accepted = new AtomicInteger();
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route, goal, true,
                    accepted::incrementAndGet, null, resources, true, true));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertTrue(controller.latest().rocket());

            // The empty hotbar is the explicit rejected-request outcome: no rocket ever attaches.
            int observed = FlightControlAdmission.REJECTED_ROCKET_OBSERVATION_TICKS;
            for (int tick = 0; tick < observed + 7; tick++) {
                movementSync.snapshotInput();
                var ownership = movementSync.flightControlOwnership();
                assertTrue(ownership.active(), "lost control at tick=" + tick);
                assertEquals(1, accepted.get(), "the rocket is requested exactly once");
                assertEquals(tick >= observed, ownership.recovering(),
                        "hand-over must start at the observation tick, tick=" + tick);
                if (tick >= observed) assertTrue(movementSync.pitch.get() == -45f
                        || movementSync.pitch.get() == 0f, "hand-over control, tick=" + tick);
                applyCurrentControl();
                assertFalse(movementSync.onGround.get());
                assertTrue(movementSync.position.get().y > LAVA_TOP,
                        "rejected request must stay clear of lava, tick=" + tick
                                + " y=" + movementSync.position.get().y);
            }
        }
    }

    private void applyCurrentControl() {
        var state = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(),
                movementSync.onGround.get(), movementSync.fallDistance.get(), PlayerPose.GLIDING);
        var result = new GlidingPhysics(movementSync.yaw.get(), movementSync.pitch.get(),
                movementSync.hasAttachedFirework(), PlayerInput.NONE,
                movementSync.getMovementParameters()).step(movementSync.getWorld(), state,
                        PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
        movementSync.position.set(result.position());
        movementSync.velocity.set(result.velocity());
        movementSync.onGround.set(result.onGround());
        movementSync.fallDistance.set(result.fallDistance());
        movementSync.completePhysicsStep();
    }
}
