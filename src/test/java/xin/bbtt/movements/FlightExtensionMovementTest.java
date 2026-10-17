package xin.bbtt.movements;

import java.lang.reflect.Field;
import java.util.List;
import org.joml.Vector3d;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.flight.FlightAction;
import xin.bbtt.flight.FlightConfiguration;
import xin.bbtt.flight.FlightContext;
import xin.bbtt.flight.FlightContextBuilder;
import xin.bbtt.flight.FlightMode;
import xin.bbtt.flight.FlightPhase;
import xin.bbtt.flight.Trajectory;
import xin.bbtt.physics.PhysicsResult;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class FlightExtensionMovementTest extends ElytraCommandTestSupport {
    @BeforeEach
    void initializeGlidingWorld() throws Exception {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
        };
        Field field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, world);
        movementSync.setRecoveryDimension("minecraft:overworld");
        movementSync.position.set(new Vector3d(0, 60, 0));
        movementSync.velocity.set(new Vector3d(0, -.1, .5));
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
    }

    @Test
    void constructorSnapshotsBuilderBeforeLaterRegistrations() throws Exception {
        var first = new RecordingMode(true);
        var later = new RecordingMode(true);
        var builder = new FlightContextBuilder().addMode(first);
        var movement = movement(builder);
        builder.addMode(later).cruiseDamageFirst(true);
        assertEquals(List.of(first), configuration(movement).modes());
        assertFalse(configuration(movement).cruiseDamageFirst());
        movement.init();
        movement.onTick();
        assertEquals(1, first.executions);
        assertEquals(0, later.candidateCalls);
        assertEquals(List.of(first, later), configuration(movement(builder)).modes());
    }

    @Test
    void actualTickExecutesSelectedPluginAgainstLiveClient() {
        var plugin = new RecordingMode(true);
        var movement = movement(new FlightContextBuilder().addMode(plugin));
        movement.init();
        assertEquals(0, plugin.executions);
        movement.onTick();
        assertEquals(1, plugin.executions);
        assertSame(movementSync, plugin.executedSync);
        assertFalse(plugin.executedContext.emergency());
        assertEquals(FlightPhase.Standard.CRUISE, plugin.executedContext.phase());
        assertEquals(23f, movementSync.pitch.get());
        assertFalse(movement.isFinished());
        assertTrue(movementSync.isGliding());
    }

    @Test
    void unavailableModeNeverSuppliesCandidates() {
        var unavailable = new RecordingMode(false);
        var available = new RecordingMode(true);
        var movement = movement(new FlightContextBuilder().addMode(unavailable).addMode(available));
        movement.init();
        movement.onTick();
        assertTrue(unavailable.availabilityCalls > 0);
        assertEquals(0, unavailable.candidateCalls);
        assertEquals(0, unavailable.executions);
        assertEquals(1, available.executions);
    }

    @Test
    void allUnavailableModesDoNotBypassRegisteredSafetyWithVanillaSteering() {
        movementSync.pitch.set(80f);
        var vanilla = new ElytraFlyMovement(new Vector3d(0, 60, 800));
        vanilla.init();
        vanilla.onTick();
        float expectedPitch = movementSync.pitch.get();
        float expectedYaw = movementSync.yaw.get();
        assertNotEquals(80f, expectedPitch);
        movementSync.pitch.set(80f);
        movementSync.yaw.set(0f);
        var first = new RecordingMode(false);
        var second = new RecordingMode(false);
        var movement = movement(new FlightContextBuilder().addMode(first).addMode(second));
        movement.init();
        assertDoesNotThrow(movement::onTick);
        assertTrue(first.availabilityCalls > 0);
        assertTrue(second.availabilityCalls > 0);
        assertEquals(0, first.candidateCalls + second.candidateCalls);
        assertEquals(0, first.executions + second.executions);
        assertEquals(80f, movementSync.pitch.get());
        assertEquals(0f, movementSync.yaw.get());
        assertTrue(movement.isFinished());
        assertTrue(movementSync.isGliding());
    }

    @Test
    void safeLandingSelectsPluginWithEmergencyContext() {
        var plugin = new RecordingMode(true);
        var movement = movement(new FlightContextBuilder().addMode(plugin)).forSafeLanding();
        movement.init();
        movement.onTick();
        assertEquals(1, plugin.executions);
        assertTrue(plugin.executedContext.emergency());
        assertEquals(FlightPhase.Standard.FORCED_LANDING, plugin.executedContext.phase());
        assertFalse(movement.isFinished());
        assertTrue(movementSync.isGliding());
    }

    @Test
    void defaultMovementSnapshotsGlobalRegistrationsAtConstruction() throws Exception {
        synchronized (FlightContextBuilder.class) {
            var before = new ElytraFlyMovement(new Vector3d(0, 60, 800));
            var plugin = new RecordingMode(true);
            FlightContextBuilder.registerGlobalMode(plugin);
            try {
                var after = new ElytraFlyMovement(new Vector3d(0, 60, 800));
                assertFalse(configuration(before).modes().contains(plugin));
                assertTrue(configuration(after).modes().contains(plugin));
            } finally {
                Field field = FlightContextBuilder.class.getDeclaredField("GLOBAL_MODES");
                field.setAccessible(true);
                ((List<?>) field.get(null)).remove(plugin);
            }
            var restored = new ElytraFlyMovement(new Vector3d(0, 60, 800));
            assertEquals(configuration(before).modes().size(), configuration(restored).modes().size());
            assertFalse(configuration(restored).modes().contains(plugin));
        }
    }

    private static ElytraFlyMovement movement(FlightContextBuilder builder) {
        return new ElytraFlyMovement(new Vector3d(0, 60, 800), builder);
    }

    private static FlightConfiguration configuration(ElytraFlyMovement movement) throws Exception {
        Field field = ElytraFlyMovement.class.getDeclaredField("flightConfiguration");
        field.setAccessible(true);
        return (FlightConfiguration) field.get(movement);
    }

    private record PluginAction() implements FlightAction {}

    private static final class RecordingMode implements FlightMode {
        private final boolean available;
        private int availabilityCalls;
        private int candidateCalls;
        private int executions;
        private MovementSync executedSync;
        private FlightContext executedContext;

        private RecordingMode(boolean available) { this.available = available; }

        @Override public boolean isAvailable(FlightContext context) {
            availabilityCalls++;
            return available;
        }

        @Override public List<? extends FlightAction> candidates(FlightContext context) {
            candidateCalls++;
            return List.of(new PluginAction());
        }

        @Override public Trajectory predict(FlightContext context, FlightAction action) {
            return new Trajectory(context.state(), List.of(new PhysicsResult(
                    context.state().position().add(0, 0, .5), new Vector3d(0, 0, .5),
                    false, false, false)));
        }

        @Override public void execute(FlightContext context, FlightAction action, MovementSync sync) {
            executions++;
            executedContext = context;
            executedSync = sync;
            sync.pitch.set(23f);
        }
    }
}
