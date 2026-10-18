package xin.bbtt.movements;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import static org.junit.jupiter.api.Assertions.*;

class RelocationDeploymentSafetyTest {
    private static final BlockState STONE = new BlockState("stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState("air", 0, Map.of(), "empty", 0, false, "AIR");
    private record Box(double x, double y, double z, double X, double Y, double Z) {}
    private static final class Inland extends World {
        final List<Box> trace = new ArrayList<>();
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public BlockState getBlockStateAt(Vector3d p) { return p.y < 33 ? STONE : AIR; }
        @Override public boolean isBoxColliding(double x, double y, double z, double X, double Y, double Z) {
            trace.add(new Box(x, y, z, X, Y, Z));
            return y < 33;
        }
    }
    private static PhysicsState state(PhysicsResult r) {
        return new PhysicsState(r.position(), r.velocity(), r.onGround(), r.fallDistance(), r.pose());
    }

    @Test void inlandRelocationExecutesAllRealDualLandingTrajectories() {
        Inland world = new Inland();
        PhysicsState initial = new PhysicsState(new Vector3d(245.5, 33, 20.5), new Vector3d(), true);
        var ordinary = DeploymentSafety.assess(world, initial, MovementParameters.DEFAULT, -90, 0);
        assertTrue(ordinary.safe(), ordinary.reason());
        assertEquals("bounded launch and early interruption branches return to safe support", ordinary.reason());
        PhysicsState deployed = initial;
        for (int tick = 0; tick < 40 && (deployed.onGround() || deployed.velocity().y > 0); tick++) {
            deployed = state(new WalkingPhysics(PlayerInput.NONE.withJump(tick == 0), -90, true,
                    MovementParameters.DEFAULT).step(world, deployed, PlayerPose.HALF_WIDTH, (double) 1.8f));
        }
        assertFalse(deployed.onGround());
        assertTrue(deployed.velocity().y <= 0);
        for (int tick = 0; tick < 5; tick++) {
            deployed = state(new GlidingPhysics(-90, 0, false, PlayerInput.NONE, MovementParameters.DEFAULT)
                    .step(world, deployed, PlayerPose.HALF_WIDTH, tick == 0 ? (double) 1.8f : (double) 0.6f));
        }
        // Trace actual collision queries from real server/client physics, not mocked outcomes.
        world.trace.clear();
        List<List<Box>> branches = new ArrayList<>();
        for (int delay : new int[]{1, 2, 4}) {
            int start = world.trace.size();
            var landing = DeploymentSafety.assessLanding(world, deployed, MovementParameters.DEFAULT, -90, 0, delay);
            assertTrue(landing.safe(), landing.reason());
            branches.add(List.copyOf(world.trace.subList(start, world.trace.size())));
        }
        assertNotEquals(branches.get(0), branches.get(1));
        assertNotEquals(branches.get(1), branches.get(2));
        List<Box> expected = List.copyOf(world.trace);
        world.trace.clear();
        var strict = DeploymentSafety.assessRelocation(world, initial, MovementParameters.DEFAULT, -90, 0);
        assertTrue(strict.safe(), strict.reason());
        int start = world.trace.size() - expected.size();
        assertTrue(start > 0, "local interruption proof must precede landing proofs");
        assertEquals(expected, world.trace.subList(start, world.trace.size()),
                "strict inland proof must execute the shared delay-1/2/4 dual trajectories in order");
    }
}
