package xin.bbtt.flight;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.physics.DamageParameters;
import xin.bbtt.physics.MovementParameters;
import xin.bbtt.physics.PhysicsResult;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class FlightExtensionContractTest {
    static final World LOADED = new World() {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
    };

    static FlightContext context() {
        return context(LOADED, new PhysicsState(new Vector3d(8, 100, 8), new Vector3d(0, 0, 1), false));
    }

    static FlightContext context(World world, PhysicsState state) {
        return new FlightContext(world, state, MovementParameters.DEFAULT, DamageParameters.NONE,
                20, new Vector3d(8, 100, 100), 0, 0, 0, false, 2, false, java.util.Map.of());
    }

    @Test void exampleOwnsPhysicsExecutionAndExtensiblePhase() {
        var mode = new LevelFlightModeExample();
        var context = context();
        var action = mode.candidates(context).get(0);
        var prediction = mode.predict(context, action);
        assertEquals(new Vector3d(8, 100, 9), prediction.finalState().position());
        assertEquals(new Vector3d(0, 0, 1), prediction.finalState().velocity());
        assertEquals(0, mode.executions, "Prediction must not execute controls");
        mode.execute(context, action, null);
        assertEquals(1, mode.executions);
        assertSame(action, mode.lastExecuted);
        assertEquals(LevelFlightModeExample.Phase.LEVEL, mode.phase(context, action));
    }

    @Test void takeoffIsExecutableAndNullLandingExplicitlyMeansUnsupported() {
        var mode = new LevelFlightModeExample();
        var takeoff = assertDoesNotThrow(() -> mode.createTakeoff(context()));
        assertNotNull(takeoff);
        takeoff.init();
        assertFalse(takeoff.isFinished());
        takeoff.onTick();
        assertTrue(takeoff.isFinished());
        assertEquals(1, mode.takeoffTicks);
        assertEquals(-1, takeoff.getTime());
        assertDoesNotThrow(takeoff::onStop);
        assertNull(mode.createLanding(context()));
    }

    @Test void contextDefensivelyCopiesTargetsStateAndInventory() {
        var target = new Vector3d(1, 2, 3);
        var position = new Vector3d(8, 100, 8);
        var velocity = new Vector3d(0, 0, 1);
        var inventory = new HashMap<String, Integer>();
        inventory.put("fuel", 2);
        var state = new PhysicsState(position, velocity, false);
        var context = new FlightContext(LOADED, state, MovementParameters.DEFAULT, DamageParameters.NONE,
                20, target, 0, 0, 0, false, 2, false, inventory);
        target.zero(); position.zero(); velocity.zero(); inventory.clear();
        context.target().zero(); context.state().position().zero(); context.state().velocity().zero();
        assertEquals(new Vector3d(1, 2, 3), context.target());
        assertEquals(new Vector3d(8, 100, 8), context.state().position());
        assertEquals(new Vector3d(0, 0, 1), context.state().velocity());
        assertEquals(2, context.inventory().get("fuel"));
        assertThrows(UnsupportedOperationException.class, () -> context.inventory().put("fuel", 9));
    }

    @Test void trajectorySnapshotsResultsAndExposesOnlyCopiedVectors() {
        var context = context();
        var position = new Vector3d(8, 100, 9);
        var results = new ArrayList<PhysicsResult>();
        results.add(new PhysicsResult(position, new Vector3d(0, 0, 1), false, false));
        var trajectory = new Trajectory(context.state(), results);
        results.clear(); position.zero();
        trajectory.results().get(0).position().zero();
        trajectory.finalState().velocity().zero();
        assertEquals(1, trajectory.results().size());
        assertEquals(new Vector3d(8, 100, 9), trajectory.finalState().position());
        assertEquals(new Vector3d(0, 0, 1), trajectory.finalState().velocity());
        assertThrows(UnsupportedOperationException.class, () -> trajectory.results().clear());
        var empty = new Trajectory(context.state(), List.of());
        assertEquals(context.state(), empty.finalState());
        assertTrue(new LevelFlightModeExample().recover(context, new LevelFlightModeExample.Level(1), trajectory).isEmpty());
    }
}
