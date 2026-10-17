package xin.bbtt.flight;

import java.util.List;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;
import xin.bbtt.physics.PhysicsResult;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class FlightTransitionsTest {
    record Action(int cost) implements FlightAction {}
    static final class Mode implements FlightMode {
        final Action action;
        boolean descending, decline, unavailable;
        int factories, recoveries;
        Trajectory recovery;
        Mode(int cost) { action = new Action(cost); }
        @Override public boolean isAvailable(FlightContext c) { return !unavailable; }
        @Override public List<? extends FlightAction> candidates(FlightContext c) { return List.of(action); }
        @Override public Trajectory predict(FlightContext c, FlightAction a) {
            return new Trajectory(c.state(), List.of(new PhysicsResult(c.state().position(),
                    new Vector3d(0, descending ? -1 : 0, 0), false, false, false)), action);
        }
        @Override public Trajectory previewTakeoff(FlightContext c) { return predict(c, action); }
        @Override public Trajectory previewLanding(FlightContext c) { return predict(c, action); }
        @Override public List<Trajectory> recover(FlightContext c, FlightAction a, Trajectory t) {
            recoveries++;
            return recovery == null ? List.of() : List.of(recovery);
        }
        @Override public void execute(FlightContext c, FlightAction a, MovementSync sync) { fail("selection executed controls"); }
        @Override public Movement createTakeoff(FlightContext c) {
            factories++;
            return decline ? null : new Movement() {
                @Override public void init() { fail("selection initialized movement"); }
                @Override public void onTick() { fail("selection ticked movement"); }
                @Override public long getTime() { return -1; }
                @Override public void onStop() {}
            };
        }
        @Override public Movement createLanding(FlightContext c) { return createTakeoff(c); }
    }

    @Test void ranksAllModesBeforeConstructingWinningTransition() {
        for (var kind : FlightTransitions.Kind.values()) {
            var first = new Mode(10);
            var best = new Mode(-10);
            var config = new FlightContextBuilder().addMode(first).addMode(best)
                    .addCostTerm((c, a, t) -> ((Action) a).cost()).build();
            var selected = FlightTransitions.select(config, FlightExtensionContractTest.context(), kind);
            assertSame(best, selected.mode());
            assertEquals(0, first.factories);
            assertEquals(1, best.factories);
        }
    }

    @Test void declinedFactoryTriesNextSafeOwner() {
        var first = new Mode(0); first.decline = true;
        var next = new Mode(1);
        var selected = FlightTransitions.select(new FlightContextBuilder().addMode(first).addMode(next).build(),
                FlightExtensionContractTest.context(), FlightTransitions.Kind.TAKEOFF);
        assertSame(next, selected.mode());
        assertEquals(1, first.factories);
    }

    @Test void unresolvedAndWrongEndpointRecoveryNeverConstructMovement() {
        var mode = new Mode(0); mode.descending = true;
        var context = FlightExtensionContractTest.context();
        var config = new FlightContextBuilder().addMode(mode).build();
        assertNull(FlightTransitions.select(config, context, FlightTransitions.Kind.TAKEOFF));
        mode.recovery = new Trajectory(context.state(), List.of(new PhysicsResult(context.state().position(),
                new Vector3d(), false, false, true)));
        assertNull(FlightTransitions.select(config, context, FlightTransitions.Kind.TAKEOFF));
        assertEquals(0, mode.factories);
        var main = mode.previewTakeoff(context);
        mode.recovery = new Trajectory(main.finalState(), mode.recovery.results());
        assertSame(mode, FlightTransitions.select(config, context, FlightTransitions.Kind.TAKEOFF).mode());
        assertEquals(3, mode.recoveries);
    }

    @Test void unknownTerrainAndUnavailableModesAreRejected() {
        var mode = new Mode(0);
        var config = new FlightContextBuilder().addMode(mode).build();
        var context = FlightExtensionContractTest.context(new World() {
            @Override public boolean chunkLoaded(int x, int z) { return false; }
        }, FlightExtensionContractTest.context().state());
        assertNull(FlightTransitions.select(config, context, FlightTransitions.Kind.LANDING));
        mode.unavailable = true;
        assertNull(FlightTransitions.select(config, FlightExtensionContractTest.context(), FlightTransitions.Kind.TAKEOFF));
        assertEquals(0, mode.factories);
    }
}
