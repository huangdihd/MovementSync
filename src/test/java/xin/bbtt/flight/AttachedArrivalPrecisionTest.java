package xin.bbtt.flight;

import java.util.ArrayList;
import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.physics.ElytraFlightController;
import xin.bbtt.physics.GlidingPhysics;
import xin.bbtt.physics.FlightHazards;
import xin.bbtt.physics.LocalAirRoute;
import xin.bbtt.physics.MovementParameters;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerInput;
import xin.bbtt.physics.PlayerPose;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

class AttachedArrivalPrecisionTest extends ElytraCommandTestSupport {
    @Test void roundedTick389SurvivesAttachmentExpiryAndLandsPrecisely() {
        World world = new World() {
            private final BlockState stone = new BlockState(
                    "minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");

            @Override public boolean chunkLoaded(int x, int z) { return true; }

            @Override public BlockState getBlockStateAt(Vector3d position) {
                return position.y < 64 || position.y >= 80
                        ? stone : super.getBlockStateAt(position);
            }
        };
        var goal = new Vector3d(16.5, 64, 608.5);
        // Rounded tick389 from terminal-precision-private/EVIDENCE.md, before thrust expires.
        // This earlier recovery does not resolve the separate late-arrival regression.
        var initial = new PhysicsState(new Vector3d(16.69, 64.67, 608.3),
                new Vector3d(1.455, .009970, -.3385), false, 1, PlayerPose.GLIDING);
        float heading = (float) Math.toDegrees(Math.atan2(
                initial.position().x - goal.x, goal.z - initial.position().z));
        var attachedArrival = LocalGotoController.arrival(world, MovementParameters.DEFAULT,
                initial, goal, true, heading);
        assertFalse(attachedArrival.isEmpty(), "attached arrival must retain a safe approach");
        var selected = attachedArrival.get(0).control();

        // A null input corridor matches the isolated evidence, not a full planner-route proof.
        var episode = LocalGotoController.prepareEpisode(world, MovementParameters.DEFAULT,
                initial, selected, true, false, 10, null, goal, false, false, null, true);
        assertEquals(10, episode.prefix().size(), "complete attached prefix");
        assertEquals(6, episode.reserve().size(), "complete reserve");
        assertEquals(1, episode.bridge().size(), "complete handoff bridge");
        var proof = LocalGotoController.attachmentExpiryProof(world, MovementParameters.DEFAULT,
                initial, episode.prefix(), episode.reserve(), episode.bridge(), goal, true, null);
        assertNotNull(proof, "every possible attachment-expiry outcome must be safe");

        var steps = new ArrayList<>(episode.prefix());
        steps.addAll(episode.reserve());
        steps.addAll(episode.bridge());
        assertEquals(steps.size() + 1, proof.branches().size());
        for (int expiryTick = 0; expiryTick <= steps.size(); expiryTick++) {
            var states = proof.branches().get(expiryTick).states();
            assertEquals(steps.size() + 1, states.size());
            assertEquals(initial, states.get(0));
            PhysicsState state = initial;
            for (int tick = 0; tick < steps.size(); tick++) {
                var next = safeStep(world, state, steps.get(tick).control(), tick < expiryTick);
                assertFalse(next.onGround(), "possible remaining thrust must not touch support");
                assertEquals(next, states.get(tick + 1),
                        "expiry=" + expiryTick + ", tick=" + tick);
                assertTrue(proof.corridor().contains(state.position(), next.position()));
                state = next;
            }
        }

        // Execute only one attached tick; subsequent controls must be replanned without thrust.
        PhysicsState state = safeStep(world, initial, selected, true);
        assertFalse(state.onGround(), "the attached tick cannot authorize touchdown");
        assertEquals(proof.branches().get(1).states().get(1), state);
        float yaw = selected.yaw();
        for (int tick = 0; tick < 40 && !state.onGround(); tick++) {
            var arrival = LocalGotoController.arrival(world, MovementParameters.DEFAULT,
                    state, goal, false, yaw);
            assertFalse(arrival.isEmpty(), "detached arrival must retain a continuation at " + state);
            var control = arrival.get(0).control();
            state = safeStep(world, state, control, false);
            yaw = control.yaw();
        }
        assertTrue(state.onGround(), "detached arrival must physically land within 40 ticks");
        assertTrue(LocalGotoController.normalLanding(world, state, goal),
                "landing must reach normal support");
        assertTrue(state.position().distance(goal) <= 1.5,
                "terminal precision must survive attachment expiry and replanning: " + state);
    }

    private static PhysicsState safeStep(World world, PhysicsState state,
            ElytraFlightController.Decision control, boolean attached) {
        var next = new GlidingPhysics(control.yaw(), control.pitch(), attached,
                PlayerInput.NONE, MovementParameters.DEFAULT)
                .step(world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
        assertTrue(LocalAirRoute.visible(world, state.position(), next.position(), PlayerPose.GLIDING),
                "transition must remain visible through loaded terrain");
        assertFalse(next.collidedX() || next.collidedZ(), "no horizontal collision");
        assertTrue(FlightHazards.safeTravel(world, state, next), "no hazardous transition");
        assertTrue(next.fallDistance() < MovementParameters.DEFAULT.safeFallDistance(),
                "every transition must preserve the strict safe-fall margin");
        return new PhysicsState(next.position(), next.velocity(), next.onGround(),
                next.fallDistance(), PlayerPose.GLIDING);
    }
}
