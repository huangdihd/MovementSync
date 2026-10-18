package xin.bbtt.movements;

import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.flight.FlightContext;
import xin.bbtt.flight.FlightContextBuilder;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

/** Actual block collision and vanilla motion, not phase-label assertions alone. */
final class PursuitTouchdownGuardTest {
    private static final BlockState STONE = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState("minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");

    private static World terrain(boolean ridge) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                int height = ridge && p.z >= 25 && p.z < 90 ? 67 : 64;
                return p.y < height ? STONE : AIR;
            }
        };
    }

    private static FlightContext context(World world, PhysicsState state, Vector3d aim,
            float yaw, float pitch, boolean attached, int rockets) {
        return new FlightContext(world, state, MovementParameters.DEFAULT, DamageParameters.NONE,
                20, aim, yaw, pitch, rockets, attached, 2, false, Map.of());
    }

    @Test void harmlessTouchdownIsNotAcceptedAsPursuitEvenWithCruiseLabel() {
        var config = new FlightContextBuilder().addStandard().build();
        var world = terrain(false);
        var state = new PhysicsState(new Vector3d(.5, 66, .5), new Vector3d(0, -.2, .4), false);
        var context = context(world, state, new Vector3d(.5, 64, 60), 0, 50, false, 8);
        var landing = new ElytraFlightController.Decision(0, 70, false, ElytraFlightController.Phase.CRUISE);
        assertNull(ElytraFlyMovement.airbornePursuitPreview(context, landing, config));
        var retained = ElytraFlyMovement.retainAirbornePursuit(context, landing, context.target(), config);
        assertNotNull(retained);
        assertNotEquals(landing, retained);
        assertNotNull(ElytraFlyMovement.airbornePursuitPreview(context, retained, config));
    }

    @Test void flatFallbackSustainsSixteenBlockPerSecondPursuitWithoutTouchdown() {
        exercise(false);
    }

    @Test void ridgeFallbackSustainsSixteenBlockPerSecondPursuitWithoutTouchdown() {
        exercise(true);
    }

    private void exercise(boolean ridge) {
        var config = new FlightContextBuilder().addStandard().build();
        var world = terrain(ridge);
        var state = new PhysicsState(new Vector3d(.5, 70, .5), new Vector3d(0, 0, .8), false);
        float yaw = 0, pitch = 0;
        int boost = 0, rockets = 30;
        for (int tick = 0; tick < 240; tick++) {
            var aim = new Vector3d(.5, 64, 32 + tick * .8);
            var context = context(world, state, aim, yaw, pitch, boost > 0, rockets);
            // Deliberately supply a terrain/route failure's downward fallback every tick.
            var fallback = new ElytraFlightController.Decision(yaw, 70, false, ElytraFlightController.Phase.FORCED_LANDING);
            var selected = ElytraFlyMovement.retainAirbornePursuit(context, fallback, aim, config);
            assertNotNull(selected, "airborne alternative at tick " + tick + " position=" + state.position());
            if (selected.firework()) { assertEquals(0, boost); boost = 20; rockets--; }
            var next = new GlidingPhysics(selected.yaw(), selected.pitch(), boost > 0).step(
                    world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            assertFalse(next.onGround(), "touchdown at tick " + tick);
            assertFalse(next.collidedX() || next.collidedZ(), "ridge impact at tick " + tick);
            state = new PhysicsState(next.position(), next.velocity(), next.onGround(), next.fallDistance(), next.pose());
            yaw = selected.yaw(); pitch = selected.pitch();
            if (boost > 0) boost--;
            assertTrue(Math.hypot(state.position().x - aim.x, state.position().z - aim.z) < 64,
                    "lost 16 blocks/sec target at tick " + tick + " position=" + state.position());
        }
        assertTrue(state.position().z > 150, "must traverse the ridge, not just hover");
    }
}
