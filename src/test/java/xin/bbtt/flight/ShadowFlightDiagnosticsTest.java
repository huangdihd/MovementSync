package xin.bbtt.flight;

import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import org.joml.Vector3d;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

class ShadowFlightDiagnosticsTest extends ElytraCommandTestSupport {
    private final ShadowFlightDiagnostics diagnostics = new ShadowFlightDiagnostics();
    private final ElytraFlightController.Decision actual = new ElytraFlightController.Decision(
            17, -8, false, ElytraFlightController.Phase.CRUISE);
    private WaypointFlightTarget route;

    @BeforeEach void airborne() {
        movementSync.position.set(new Vector3d(0, 100, 0));
        movementSync.velocity.set(new Vector3d(0, 0, .3));
        movementSync.onGround.set(false);
        movementSync.yaw.set(actual.yaw());
        movementSync.pitch.set(actual.pitch());
        route = new WaypointFlightTarget(List.of(new Vector3d(0, 100, 0),
                new Vector3d(0, 100, 40)), new Vector3d(0, 100, 40), false);
    }

    private void world(World world) throws Exception {
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, world);
    }

    private void awaitIdle() throws Exception {
        assertTrue(diagnostics.awaitIdle(5, TimeUnit.SECONDS));
    }

    private static final class GatedWorld extends World {
        final CountDownLatch entered = new CountDownLatch(1);
        final CountDownLatch release = new CountDownLatch(1);
        final AtomicInteger queries = new AtomicInteger();
        volatile Thread worker;
        @Override public boolean chunkLoaded(int x, int z) {
            queries.incrementAndGet();
            worker = Thread.currentThread();
            entered.countDown();
            try {
                if (!release.await(5, TimeUnit.SECONDS)) throw new AssertionError("Worker gate timed out");
            } catch (InterruptedException error) {
                Thread.currentThread().interrupt();
                throw new AssertionError(error);
            }
            return true;
        }
    }

    @Test void defaultDisabledDoesNotRunWorldQueries() throws Exception {
        var world = new GatedWorld();
        world(world);
        assertFalse(movementSync.getFlightScopeRouter().isShadowEnabled());
        diagnostics.submit(movementSync, FlightControlAdmission.snapshot(movementSync), route, actual);
        awaitIdle();
        assertEquals(0, world.queries.get());
        assertNull(diagnostics.latest());
    }

    @Test void enabledWorkerProducesComparisonWithoutChangingLiveControlsOrClock() throws Exception {
        var world = new GatedWorld();
        world.release.countDown();
        world(world);
        var input = movementSync.snapshotInput();
        var source = FlightControlAdmission.snapshot(movementSync);
        var position = new Vector3d(movementSync.position.get());
        var velocity = new Vector3d(movementSync.velocity.get());
        movementSync.getFlightScopeRouter().setShadowEnabled(true);
        diagnostics.submit(movementSync, source, route, actual);
        awaitIdle();
        var result = diagnostics.latest();
        assertNotNull(result);
        assertTrue(world.queries.get() > 0);
        assertNotSame(Thread.currentThread(), world.worker);
        assertEquals("flight-shadow", world.worker.getName());
        assertEquals(source.tick(), result.tick());
        assertEquals(actual.yaw(), result.actualYaw());
        assertEquals(actual.pitch(), result.actualPitch());
        assertTrue(result.simulatedSteps() >= 0 && result.simulatedSteps() <= 600);
        assertEquals(actual.yaw(), movementSync.yaw.get());
        assertEquals(actual.pitch(), movementSync.pitch.get());
        assertEquals(input, movementSync.snapshotInput());
        assertEquals(source.tick(), movementSync.physicsTickVersion());
        assertEquals(position, movementSync.position.get());
        assertEquals(velocity, movementSync.velocity.get());
        assertNull(movementSync.pollBeforeMovementPacket());
    }

    @Test void busySubmissionsAreDroppedAndCompletedTickIsNotRepeated() throws Exception {
        var world = new GatedWorld();
        world(world);
        movementSync.getFlightScopeRouter().setShadowEnabled(true);
        var source = FlightControlAdmission.snapshot(movementSync);
        diagnostics.submit(movementSync, source, route, actual);
        try {
            assertTrue(world.entered.await(5, TimeUnit.SECONDS));
            for (int i = 1; i <= 20; i++) {
                diagnostics.submit(movementSync, new FlightControlAdmission.Source(source.tick() + i * 10,
                        source.generation(), source.state(), source.epoch()), route, actual);
            }
        } finally {
            world.release.countDown();
            awaitIdle();
        }
        var result = diagnostics.latest();
        assertNotNull(result);
        assertEquals(source.tick(), result.tick(), "Busy observations must not queue");
        int queries = world.queries.get();
        diagnostics.submit(movementSync, source, route, actual);
        awaitIdle();
        assertSame(result, diagnostics.latest());
        assertEquals(queries, world.queries.get(), "Same tick must not run again");
        diagnostics.submit(movementSync, new FlightControlAdmission.Source(source.tick() + 10,
                source.generation(), source.state(), source.epoch()), route, actual);
        awaitIdle();
        assertEquals(source.tick() + 10, diagnostics.latest().tick());
    }

    @Test void disablingWhileWorkerRunsDiscardsResult() throws Exception {
        var world = new GatedWorld();
        world(world);
        movementSync.getFlightScopeRouter().setShadowEnabled(true);
        diagnostics.submit(movementSync, FlightControlAdmission.snapshot(movementSync), route, actual);
        try {
            assertTrue(world.entered.await(5, TimeUnit.SECONDS));
            movementSync.getFlightScopeRouter().setShadowEnabled(false);
        } finally {
            world.release.countDown();
            awaitIdle();
        }
        assertNull(diagnostics.latest());
    }

    @Test void emptyVisibleLookaheadPublishesNoSafeCandidate() throws Exception {
        world(new World() {
            @Override public boolean chunkLoaded(int x, int z) { return false; }
        });
        movementSync.getFlightScopeRouter().setShadowEnabled(true);
        diagnostics.submit(movementSync, FlightControlAdmission.snapshot(movementSync), route, actual);
        awaitIdle();
        var result = diagnostics.latest();
        assertNotNull(result);
        assertEquals("NO_SAFE_CANDIDATE", result.status());
        assertEquals(0, result.simulatedSteps());
        assertNull(result.shadowYaw());
        assertNull(result.shadowPitch());
    }
}
