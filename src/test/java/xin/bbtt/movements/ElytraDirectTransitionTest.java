package xin.bbtt.movements;

import java.util.List;
import org.joml.Vector3d;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.flight.*;
import xin.bbtt.movement.Movement;
import xin.bbtt.physics.PhysicsResult;

import static org.junit.jupiter.api.Assertions.*;

final class ElytraDirectTransitionTest extends ElytraCommandTestSupport {
    @BeforeEach void loadedWorld() throws Exception {
        movementSync.getMovementController().pause();
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, new xin.bbtt.world.World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
        });
    }

    final class Mode implements FlightMode {
        final FlightAction action = new FlightAction() {};
        int starts, ticks, stops, executions;
        boolean landing;
        @Override public List<? extends FlightAction> candidates(FlightContext c) { return List.of(action); }
        @Override public Trajectory predict(FlightContext c, FlightAction a) {
            return new Trajectory(c.state(), List.of(new PhysicsResult(c.state().position(),
                    new Vector3d(), false, false, landing)), a);
        }
        @Override public Trajectory previewTakeoff(FlightContext c) { return landing ? null : predict(c, action); }
        @Override public Trajectory previewLanding(FlightContext c) { return landing ? predict(c, action) : null; }
        @Override public Movement createTakeoff(FlightContext c) { return child(); }
        @Override public Movement createLanding(FlightContext c) { return child(); }
        @Override public void execute(FlightContext c, FlightAction a, MovementSync sync) { executions++; }
        Movement child() {
            return new Movement() {
                @Override public void init() { starts++; }
                @Override public void onTick() {
                    ticks++;
                    movementSync.onGround.set(landing);
                    setFinished(true);
                }
                @Override public void onStop() { stops++; }
                @Override public long getTime() { return -1; }
            };
        }
    }

    @Test void takeoffPrecedesVanillaJumpAndHandsOffNonGlidingMode() {
        var mode = new Mode();
        movementSync.onGround.set(true);
        var flight = new ElytraFlyMovement(new Vector3d(200, 0, 0), new FlightContextBuilder().addMode(mode));
        flight.init();
        assertEquals(1, mode.starts);
        assertFalse(movementSync.snapshotInput().jump());
        flight.onTick();
        assertEquals(1, mode.ticks);
        assertEquals(1, mode.stops);
        assertFalse(movementSync.isGliding());
        flight.onTick();
        assertEquals(1, mode.executions);
        assertFalse(flight.isFinished());
        flight.onStop();
        assertEquals(1, mode.stops);
    }

    @Test void arrivalAndSafeLandingUsePluginChild() {
        for (boolean safe : new boolean[]{false, true}) {
            var mode = new Mode();
            mode.landing = true;
            movementSync.onGround.set(false);
            var flight = new ElytraFlyMovement(new Vector3d(safe ? 200 : 0, 0, 0),
                    new FlightContextBuilder().addMode(mode));
            if (safe) flight.forSafeLanding();
            flight.init();
            flight.onTick();
            assertEquals(1, mode.starts);
            assertEquals(0, mode.executions);
            flight.onTick();
            assertEquals(1, mode.ticks);
            assertEquals(1, mode.stops);
            flight.onTick();
            assertTrue(flight.isFinished());
        }
    }

    @Test void supersededRequestStopsChildWithoutTickingIt() {
        var mode = new Mode();
        var flight = new ElytraFlyMovement(new Vector3d(200, 0, 0), new FlightContextBuilder().addMode(mode));
        flight.init();
        movementSync.beginGotoNavigationRequest(new org.joml.Vector3i(300, 0, 0), false, false);
        flight.onTick();
        assertTrue(flight.isFinished());
        assertEquals(1, mode.stops);
        assertEquals(0, mode.ticks);
    }

    @Test void cancellationStopsChildWithoutTickingOrClearingGlide() {
        var mode = new Mode();
        var flight = new ElytraFlyMovement(new Vector3d(200, 0, 0), new FlightContextBuilder().addMode(mode));
        flight.init();
        movementSync.setGliding(true);
        flight.onStop();
        flight.onStop();
        flight.onTick();
        assertEquals(1, mode.stops);
        assertEquals(0, mode.ticks);
        assertTrue(movementSync.isGliding());
    }
}
