package xin.bbtt.movements;

import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.flight.*;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

/** Actual gliding integration, not kinematic position updates or mocked trajectory results. */
final class LowAltitudeLavaRescuePhysicsTest {
    private static final BlockState AIR = block("air", false);
    private static final BlockState LAVA = block("lava", false);
    private static final BlockState STONE = block("stone", true);

    private static BlockState block(String name, boolean solid) {
        return new BlockState("minecraft:" + name, solid ? 1 : 0, Map.of(),
                solid ? "block" : "empty", solid ? 1.5 : 0, solid, "");
    }

    private static World terrain() {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return x >= 28 && x <= 36 && z >= -4 && z <= 4; }
            @Override public java.util.Set<ChunkPosition> loadedChunks() {
                var chunks = new java.util.HashSet<ChunkPosition>();
                for (int x = 28; x <= 36; x++) for (int z = -4; z <= 4; z++)
                    chunks.add(new ChunkPosition(x, z));
                return java.util.Set.copyOf(chunks);
            }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                // Lava sea terminates at a wall/plateau; sideways shores remain available.
                if (p.y >= 60 || p.y < 29 || p.x >= 500 && p.y < 42
                        || Math.abs(p.z) >= 8 && p.y < 32) return STONE;
                return p.y < 32 ? LAVA : AIR;
            }
        };
    }

    @Test void stalledFeet32Point5AboveLava31TurnsOrClimbsWithoutEnteringLava() {
        exercise(new Vector3d(0, 0, 0));
    }

    @Test void lavaSeaWallAndPlateauPreviewSurvivesInjectedVelocityLoss() {
        exercise(new Vector3d(.18, -.02, 0));
    }

    private static void exercise(Vector3d velocity) {
        World world = terrain();
        var state = new PhysicsState(new Vector3d(499, 32.5, .5), velocity, false);
        assertTrue(ElytraFlyMovement.harmBelow(world, state));
        // Inject complete velocity loss, rather than preserving the approach's momentum.
        state = new PhysicsState(state.position(), new Vector3d(), false, 0, PlayerPose.GLIDING);
        replayRecovery(world, state, new int[] {1, 1, 1, 1, 1, 1, 1, 1});
    }

    @Test void distant180BlockCoastHasExecutablePoweredRouteAndTerminalLanding() {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) {
                return x >= -2 && x <= 16 && z >= -2 && z <= 2;
            }
            @Override public java.util.Set<ChunkPosition> loadedChunks() {
                var chunks = new java.util.HashSet<ChunkPosition>();
                for (int x = -2; x <= 16; x++) for (int z = -2; z <= 2; z++)
                    chunks.add(new ChunkPosition(x, z));
                return java.util.Set.copyOf(chunks);
            }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.y >= 48) return STONE;
                if (p.y < 32) return p.x >= 180 ? STONE : LAVA;
                return AIR;
            }
        };
        var state = new PhysicsState(new Vector3d(.5, 40, .5), new Vector3d(), false,
                0, PlayerPose.GLIDING);
        int[] durations = new int[12];
        java.util.Arrays.fill(durations, 1);
        var resources = new HazardRecovery.Context(durations, 0, false, null);
        assertNotNull(LowAltitudePlanner.plan(world, state, new Vector3d(184.5, 32, .5),
                MovementParameters.DEFAULT, resources), "powered route must not be rejected as a coast-only glide");
        replayRecovery(world, state, durations);
    }

    private static void replayRecovery(World world, PhysicsState state, int[] durations) {
        var report = HazardRecovery.assess(world, state, MovementParameters.DEFAULT,
                new HazardRecovery.Context(durations, 0, false, null));
        assertEquals(HazardRecovery.Status.RECOVERABLE, report.status(), report.reason());
        assertNotNull(report.decision(), "recovery must supply executable first controls");
        assertNotNull(report.target(), "recovery must identify terminal support");
        var decision = report.decision();
        int used = 0, cooldown = 0;
        long boost = 0;
        boolean landed = false;
        for (int tick = 0; tick < 600; tick++) {
            // Replay the finite resource schedule used by recovery, never an infinite attachment.
            if (decision.firework() && boost == 0 && cooldown == 0 && used < durations.length) {
                // Minimum vanilla lifetime includes the expiry tick's acceleration.
                boost = 10L * (1 + durations[used++]) + 1;
                cooldown = 40;
            }
            var context = new FlightContext(world, state, MovementParameters.DEFAULT,
                    DamageParameters.NONE, 20, report.target(), decision.yaw(), decision.pitch(),
                    durations.length - used, boost > 0, 2, true, Map.of());
            var result = new GlidingPhysics(decision.yaw(), decision.pitch(), boost > 0).step(
                    world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            assertEquals(0, PublicFlightBridge.hazard(context, state, result, true).raw());
            assertEquals(0, PublicFlightBridge.hazard(context, state, result, false).raw());
            assertFalse(result.collidedX() || result.collidedZ());
            assertTrue(result.position().y >= 32, "never enter lava");
            state = new PhysicsState(result.position(), result.velocity(), result.onGround(),
                    result.fallDistance(), result.pose());
            boost = Math.max(0, boost - 1);
            cooldown = Math.max(0, cooldown - 1);
            if (result.onGround()) { landed = true; break; }
        }
        assertTrue(landed, "rescue must end on support, not merely survive an airborne prefix");
        assertTrue(world.isOnGround(state.position()));
        assertEquals(0, state.position().distance(report.target()), 1.e-6);
        assertTrue(used > 0 && used <= durations.length);
    }

    @Test void feetAtLavaSurfaceCannotBeRescuedEvenWithRockets() {
        var world = terrain();
        var state = new PhysicsState(new Vector3d(499, 32, .5), new Vector3d(), false);
        var context = new FlightContext(world, state, MovementParameters.DEFAULT, DamageParameters.NONE,
                20, new Vector3d(520, 42, .5), -90, 0, 8, false, 2, true, Map.of());
        // Rocket acceleration follows this tick's movement: no pitch can prevent the initial sink.
        assertFalse(HazardRecovery.assess(world, state, MovementParameters.DEFAULT).admissible());
        assertNull(LowAltitudePlanner.plan(world, state, new Vector3d(520, 42, .5),
                MovementParameters.DEFAULT), "unrecoverable initial state must not become an executable route");
        assertNull(ElytraFlyMovement.lowHazardDecision(context,
                new FlightContextBuilder().addStandard().build()));
    }

    @Test void noRocketCannotInventPoweredRescueAtZeroSpeed() {
        var world = terrain();
        var state = new PhysicsState(new Vector3d(499, 32, .5), new Vector3d(), false);
        var context = new FlightContext(world, state, MovementParameters.DEFAULT, DamageParameters.NONE,
                20, new Vector3d(520, 42, .5), -90, 0, 0, false, 2, true, Map.of());
        assertNull(ElytraFlyMovement.lowHazardDecision(context,
                new FlightContextBuilder().addStandard().build()));
    }
}
