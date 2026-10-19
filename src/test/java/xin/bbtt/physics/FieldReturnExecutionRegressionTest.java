package xin.bbtt.physics;

import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Rounded field tick516; synthetic loaded lava sea, not a captured terrain claim. */
class FieldReturnExecutionRegressionTest extends ElytraCommandTestSupport {
    private World shore() {
        BlockState sand = null;
        for (int id = 0; id < xin.bbtt.Block.BlockStateParser.getBlockStateRegistrySize(); id++) {
            var block = xin.bbtt.Block.BlockStateParser.Instance.parseStateId(id);
            if (block.blockName().equals("soul_sand")) { sand = block; break; }
        }
        assertNotNull(sand);
        final BlockState support = sand;
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.x < 250 && p.y < 34) return support;
                if (p.y < 31) return new BlockState("minecraft:lava", 0,
                        Map.of(), "empty", 0, false, "LAVA");
                return super.getBlockStateAt(p);
            }
        };
    }

    @Test void delayedPublicationReplaysFixedReturnBeforeDispatchingRocket() throws Exception {
        checkPublication(false);
    }

    @Test void delayedPublicationCannotDispatchFromLava() throws Exception {
        checkPublication(true);
    }

    private void checkPublication(boolean unsafe) throws Exception {
        var world = shore();
        var field = xin.bbtt.MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, world);
        inventory[36] = item("firework_rocket", 8);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        movementSync.position.set(new Vector3d(376.4, 32.46, 12.73));
        movementSync.velocity.set(new Vector3d(.3791, -.08101, -.01297));
        movementSync.fallDistance.set(1.0810050731796323);
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        Object owner = new Object();
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        var source = FlightControlAdmission.snapshot(movementSync);
        var flights = movementSync.inventoryManager.getHotbarFireworkFlightDurations();
        var anchor = VanillaGlideMode.landingSite(world, 229.8, 20.75, 33.9);
        assertNotNull(anchor);
        var plan = KnownReturnPlan.preview(world, source.state(), MovementParameters.DEFAULT,
                new HazardRecovery.Context(flights, 0, false, null), anchor, 0);
        assertTrue(plan.proven(), plan.reason());
        assertTrue(plan.steps().stream().anyMatch(s -> s.control().firework()),
                "Unpowered touchdown does not mean unpowered return");
        assertTrue(plan.steps().get(0).control().firework());
        // Simulate two outward unpowered ticks spent preparing the event preview.
        var state = source.state();
        for (int i = 0; i < 2; i++) {
            state = FlightSafety.nextState(new GlidingPhysics(-90, -20, false,
                    PlayerInput.NONE, MovementParameters.DEFAULT).step(world, state,
                    PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height()));
            movementSync.completePhysicsStep();
        }
        if (unsafe) state = new PhysicsState(new Vector3d(376.4, 30.5, 12.73), state.velocity(), false,
                state.fallDistance(), PlayerPose.GLIDING);
        var replay = KnownReturnPlan.replay(world, state, MovementParameters.DEFAULT, plan);
        assertEquals(!unsafe, replay.proven(), replay.reason());
        if (!unsafe) {
            var replayState = state;
            for (var step : replay.steps()) {
                assertEquals(replayState, step.before());
                var next = new GlidingPhysics(step.control().yaw(), step.control().pitch(),
                        step.resource().powered(), PlayerInput.NONE, MovementParameters.DEFAULT)
                        .step(world, replayState, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
                assertEquals(0, FlightHazards.damage(world, replayState, next, DamageParameters.NONE, true).raw());
                replayState = FlightSafety.nextState(next);
                assertEquals(step.after(), replayState);
            }
            assertTrue(replayState.onGround());
            assertEquals(33.875, replayState.position().y);
            assertFalse(replay.steps().get(replay.steps().size() - 1).resource().powered());
        }
        movementSync.position.set(state.position());
        movementSync.velocity.set(state.velocity());
        movementSync.fallDistance.set(state.fallDistance());
        var requests = new AtomicInteger();
        movementSync.submitFlightControls(FlightControlAdmission.knownReturn(owner, source, plan, flights,
                decision -> { assertTrue(movementSync.useFirework()); requests.incrementAndGet(); }));
        movementSync.snapshotInput();
        if (unsafe) {
            assertEquals(0, requests.get());
            assertNotNull(movementSync.getFlightControlFailure());
            return;
        }
        assertEquals(1, requests.get(), "A safe fixed replay must dispatch its first rocket, not expire forever");
        movementSync.snapshotInput();
        assertEquals(1, requests.get(), "One request per physical tick");
        assertNull(movementSync.getFlightControlFailure());
        assertNotNull(movementSync.pollBeforeMovementPacket());
    }
}
