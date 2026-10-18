package xin.bbtt.flight;

import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.FlightControlAdmission;
import xin.bbtt.physics.HazardRecovery;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

class ReserveMonitorTest extends ElytraCommandTestSupport {
    private final ReserveMonitor monitor = new ReserveMonitor();
    private static final BlockState STONE = new BlockState("minecraft:stone", 1,
            Map.of(), "block", 1.5, true, "STONE");

    private final class GatedWorld extends World {
        final CountDownLatch entered = new CountDownLatch(1);
        final CountDownLatch release = new CountDownLatch(1);
        final AtomicInteger queries = new AtomicInteger();
        volatile Thread worker;
        volatile boolean physicalLockHeld;
        @Override public boolean chunkLoaded(int x, int z) {
            worker = Thread.currentThread();
            physicalLockHeld |= Thread.holdsLock(movementSync);
            queries.incrementAndGet();
            entered.countDown();
            try {
                if (!release.await(5, TimeUnit.SECONDS)) throw new AssertionError("Gate timed out");
            } catch (InterruptedException error) {
                Thread.currentThread().interrupt();
                throw new AssertionError(error);
            }
            return true;
        }
        @Override public BlockState getBlockStateAt(Vector3d point) { return STONE; }
    }

    private void world(World world) throws Exception {
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, world);
        movementSync.position.set(new Vector3d(8, 100, 8));
    }

    private FlightControlAdmission.Source source() { return FlightControlAdmission.snapshot(movementSync); }
    private boolean observe(long token, HazardRecovery.Context context) {
        return monitor.observe(movementSync, source(), token, context);
    }
    private void idle() throws Exception { assertTrue(monitor.awaitIdle(5, TimeUnit.SECONDS)); }

    @Test void defaultWorkerIsAsyncDaemonBoundedAndNeverChangesControls() throws Exception {
        var world = new GatedWorld();
        world(world);
        var before = source();
        var input = movementSync.getInput();
        var yaw = movementSync.yaw.get();
        var pitch = movementSync.pitch.get();
        assertEquals(ReserveMonitor.Status.UNKNOWN, monitor.latest().status());
        assertTrue(observe(0, HazardRecovery.Context.EMPTY));
        try {
            assertTrue(world.entered.await(5, TimeUnit.SECONDS));
            assertNotSame(Thread.currentThread(), world.worker);
            assertTrue(world.worker.isDaemon());
            assertEquals("flight-reserve", world.worker.getName());
            assertFalse(world.physicalLockHeld);
            // The same global gate also prevents a second monitor from queuing work.
            var other = new ReserveMonitor();
            for (int i = 0; i < 30; i++) {
                assertFalse(observe(0, HazardRecovery.Context.EMPTY));
                assertFalse(other.observe(movementSync, source(), i, HazardRecovery.Context.EMPTY));
            }
            assertEquals(1, world.queries.get());
        } finally {
            world.release.countDown();
            idle();
        }
        var result = monitor.latest();
        assertEquals(ReserveMonitor.Status.SAFE, result.status());
        assertEquals(before.tick(), result.tick());
        assertEquals(before.generation(), result.generation());
        assertEquals(before.epoch(), result.epoch());
        assertTrue(result.simulatedSteps() <= HazardRecovery.DECISION_STEP_BUDGET);
        assertEquals(before, source());
        assertEquals(input, movementSync.getInput());
        assertEquals(yaw, movementSync.yaw.get());
        assertEquals(pitch, movementSync.pitch.get());
        assertNull(movementSync.pollBeforeMovementPacket());
        int queries = world.queries.get();
        for (int i = 0; i < 9; i++) {
            movementSync.completePhysicsStep();
            assertFalse(observe(0, HazardRecovery.Context.EMPTY));
        }
        assertEquals(queries, world.queries.get());
        movementSync.completePhysicsStep();
        assertEquals(ReserveMonitor.Status.STALE, monitor.latest().status());
        assertTrue(observe(0, HazardRecovery.Context.EMPTY));
        idle();
        assertEquals(before.tick() + 10, monitor.latest().tick());
    }

    @Test void eventsAndResourceChangesBypassCadenceEvenOnSameTick() throws Exception {
        var world = new GatedWorld();
        world.release.countDown();
        world(world);
        assertTrue(observe(0, HazardRecovery.Context.EMPTY));
        idle();
        // Caller tokens represent route/off-route/health changes.
        for (long token = 1; token <= 3; token++) {
            assertTrue(observe(token, HazardRecovery.Context.EMPTY));
            idle();
        }
        var stocked = new HazardRecovery.Context(new int[] {1}, 0, false, null);
        assertTrue(observe(3, stocked));
        idle();
        var attached = new HazardRecovery.Context(new int[] {1}, 40, true, null, 21);
        assertTrue(observe(3, attached));
        idle();
        assertFalse(observe(3, new HazardRecovery.Context(new int[] {1}, 39, true, null, 20)),
                "Ordinary countdown is not an immediate resource event");
        assertTrue(observe(3, stocked));
        idle();
    }

    @Test void busyEventInvalidatesInFlightProofWithoutQueuingAndCanBeRetried() throws Exception {
        var world = new GatedWorld();
        world(world);
        assertTrue(observe(0, HazardRecovery.Context.EMPTY));
        try {
            assertTrue(world.entered.await(5, TimeUnit.SECONDS));
            assertFalse(observe(1, HazardRecovery.Context.EMPTY));
            assertEquals(1, world.queries.get());
        } finally {
            world.release.countDown();
            idle();
        }
        assertEquals(ReserveMonitor.Status.STALE, monitor.latest().status());
        assertTrue(observe(1, HazardRecovery.Context.EMPTY));
        idle();
        assertEquals(ReserveMonitor.Status.SAFE, monitor.latest().status());
    }

    @Test void epochChangeDiscardsRunningResult() throws Exception {
        var world = new GatedWorld();
        world(world);
        assertTrue(observe(0, HazardRecovery.Context.EMPTY));
        try {
            assertTrue(world.entered.await(5, TimeUnit.SECONDS));
            movementSync.invalidateFlightControls();
        } finally {
            world.release.countDown();
            idle();
        }
        assertEquals(ReserveMonitor.Status.STALE, monitor.latest().status());
    }

    @Test void worldChangeDiscardsRunningResult() throws Exception {
        var world = new GatedWorld();
        world(world);
        assertTrue(observe(0, HazardRecovery.Context.EMPTY));
        try {
            assertTrue(world.entered.await(5, TimeUnit.SECONDS));
            world(new World());
        } finally {
            world.release.countDown();
            idle();
        }
        assertEquals(ReserveMonitor.Status.STALE, monitor.latest().status());
    }

    @Test void generationChangeDiscardsRunningResult() throws Exception {
        var world = new GatedWorld();
        world(world);
        assertTrue(observe(0, HazardRecovery.Context.EMPTY));
        try {
            assertTrue(world.entered.await(5, TimeUnit.SECONDS));
            var field = MovementSync.class.getDeclaredField("navigationGeneration");
            field.setAccessible(true);
            ((java.util.concurrent.atomic.AtomicLong) field.get(movementSync)).incrementAndGet();
        } finally {
            world.release.countDown();
            idle();
        }
        assertEquals(ReserveMonitor.Status.STALE, monitor.latest().status());
    }

    @Test void unknownTerrainIsUnknownNotDefinitivelyUnsafe() throws Exception {
        world(new World());
        assertTrue(observe(0, HazardRecovery.Context.EMPTY));
        idle();
        assertEquals(ReserveMonitor.Status.UNKNOWN, monitor.latest().status());
    }

    @Test void physicalMonitorCallerNeverStartsSearch() throws Exception {
        var world = new GatedWorld();
        world(world);
        synchronized (movementSync) {
            assertFalse(observe(0, HazardRecovery.Context.EMPTY));
        }
        idle();
        assertEquals(0, world.queries.get());
    }
}
