package xin.bbtt.flight;

import java.util.List;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.PhysicsResult;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class FlightSelectorExtensionTest {
    @Test void mixedVanillaAndPluginCandidatesSelectAndDispatchToOwningMode() {
        var plugin = new LevelFlightModeExample();
        var context = FlightExtensionContractTest.context();
        var selector = new FlightSelector(new FlightContextBuilder().addStandard().addMode(plugin)
                .addCostTerm((c, a, t) -> a instanceof LevelFlightModeExample.Level ? -1e9 : 0).build());
        var selected = selector.select(context);
        assertSame(plugin, selected.mode());
        assertInstanceOf(LevelFlightModeExample.Level.class, selected.action());
        assertEquals(LevelFlightModeExample.Phase.LEVEL, selected.phase());
        assertEquals(0, plugin.executions);
        selected.mode().execute(context, selected.action(), null);
        assertEquals(1, plugin.executions);
        assertSame(selected.action(), plugin.lastExecuted);
    }

    @Test void wallAndFallDamageNeverWinOnCost() {
        var wallContext = FlightExtensionContractTest.context(FlightExtensionContractTest.LOADED,
                new PhysicsState(new Vector3d(8, 100, 8), new Vector3d(2, 0, 0), false));
        var wall = new ScriptedMode(new Vector3d(), true, false);
        assertDamagingLosesToSafe(wallContext, wall);
        var fallContext = FlightExtensionContractTest.context(FlightExtensionContractTest.LOADED,
                new PhysicsState(new Vector3d(8, 100, 8), new Vector3d(0, -1, 0), false, 12));
        var fall = new ScriptedMode(new Vector3d(), false, true);
        assertDamagingLosesToSafe(fallContext, fall);
    }

    private static void assertDamagingLosesToSafe(FlightContext context, ScriptedMode risky) {
        var safe = new LevelFlightModeExample();
        var builder = new FlightContextBuilder().addMode(risky).addMode(safe)
                .addDamageRule(BuiltinDamageRule.WALL).addDamageRule(BuiltinDamageRule.FALL)
                .addCostTerm((c, a, t) -> a instanceof Risky ? -100 : 0);
        var costOnly = new FlightSelector(builder.build());
        var safety = costOnly.safety(context, risky, new Risky(), risky.predict(context, new Risky()));
        assertEquals(FlightSelector.DamageClass.DAMAGING, safety.damageClass());
        assertTrue(safety.raw() > 0);
        assertSame(safe, costOnly.select(context).mode(), "Cost-only cruise never lets a damaging candidate win");
        var riskyOnly = new FlightSelector(new FlightContextBuilder().addMode(risky)
                .addDamageRule(BuiltinDamageRule.WALL).addDamageRule(BuiltinDamageRule.FALL).build());
        assertThrows(IllegalStateException.class, () -> riskyOnly.select(context),
                "Without a safe candidate cruise hands over to emergency ranking");
        var damageFirst = new FlightSelector(builder.cruiseDamageFirst(true).build());
        assertSame(safe, damageFirst.select(context).mode());
    }

    @Test void unresolvedRecoveryAndUnknownTerrainCannotWinEvenWithBestCost() {
        var descending = new ScriptedMode(new Vector3d(0, -1, 0), false, false);
        var safe = new LevelFlightModeExample();
        var selector = new FlightSelector(new FlightContextBuilder().addMode(descending).addMode(safe)
                .addCostTerm((c, a, t) -> a instanceof Risky ? -100 : 0).build());
        var context = FlightExtensionContractTest.context();
        assertEquals(FlightSelector.DamageClass.UNKNOWN,
                selector.safety(context, descending, new Risky(), descending.predict(context, new Risky())).damageClass());
        assertSame(safe, selector.select(context).mode());
        World unloaded = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return false; }
        };
        var unknown = FlightExtensionContractTest.context(unloaded, context.state());
        assertThrows(IllegalStateException.class, () -> selector.select(unknown));
    }

    @Test void recoveryMustBelongToModeAndStartAtPredictionEndpoint() {
        var context = FlightExtensionContractTest.context();
        var mode = new ScriptedMode(new Vector3d(0, -1, 0), false, false);
        var selector = new FlightSelector(new FlightContextBuilder().addMode(mode).build());
        var main = mode.predict(context, new Risky());
        mode.recovery = new Trajectory(context.state(), List.of(
                new PhysicsResult(new Vector3d(8, 100, 8), new Vector3d(), false, false, true)));
        assertEquals(FlightSelector.DamageClass.UNKNOWN, selector.safety(context, mode, new Risky(), main).damageClass());
        mode.recovery = new Trajectory(main.finalState(), List.of(
                new PhysicsResult(main.finalState().position(), new Vector3d(), false, false, true)));
        assertEquals(FlightSelector.DamageClass.SAFE, selector.safety(context, mode, new Risky(), main).damageClass());
        assertSame(mode, selector.select(context).mode());
    }

    @Test void emergencyAlwaysRanksDamageFirstAndKeepsRawDamageWhenMitigated() {
        var base = FlightExtensionContractTest.context(FlightExtensionContractTest.LOADED,
                new PhysicsState(new Vector3d(8, 100, 8), new Vector3d(2, 0, 0), false));
        var context = new FlightContext(base.world(), base.state(), base.movement(),
                new xin.bbtt.physics.DamageParameters(0, 0, 4), 20, base.target(), 0, 0,
                0, false, 2, true, java.util.Map.of());
        var risky = new ScriptedMode(new Vector3d(), true, false);
        var safe = new LevelFlightModeExample();
        var selector = new FlightSelector(new FlightContextBuilder().addMode(risky).addMode(safe)
                .addDamageRule(BuiltinDamageRule.WALL).addCostTerm((c, a, t) -> a instanceof Risky ? -100 : 0).build());
        var damage = selector.safety(context, risky, new Risky(), risky.predict(context, new Risky()));
        assertEquals(FlightSelector.DamageClass.DAMAGING, damage.damageClass());
        assertEquals(0, damage.reduced());
        assertSame(safe, selector.select(context).mode());
    }

    @Test void customDamageRulesAndCostsAreAdditive() {
        var context = FlightExtensionContractTest.context();
        var mode = new LevelFlightModeExample();
        var selector = new FlightSelector(new FlightContextBuilder().addMode(mode)
                .addDamageRule((c, a, before, after) -> new DamageRule.Damage(2, 1))
                .addDamageRule((c, a, before, after) -> new DamageRule.Damage(3, 2)).build());
        var action = mode.candidates(context).getFirst();
        var safety = selector.safety(context, mode, action, mode.predict(context, action));
        assertEquals(5, safety.raw());
        assertEquals(3, safety.reduced());
        var other = new ScriptedMode(new Vector3d(), false, false);
        var costs = new FlightContextBuilder().addMode(mode).addMode(other)
                .addCostTerm((c, a, t) -> a instanceof Risky ? -10 : 0)
                .addCostTerm((c, a, t) -> a instanceof Risky ? 20 : 0);
        assertSame(mode, costs.build().select(context).mode());
    }

    @Test void sweptObstaclesRejectEndpointOnlyPluginPredictions() {
        World solid = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double a, double b, double c, double d, double e, double f) { return true; }
        };
        var mode = new LevelFlightModeExample();
        var context = FlightExtensionContractTest.context(solid, FlightExtensionContractTest.context().state());
        var selector = new FlightSelector(new FlightContextBuilder().addMode(mode).build());
        assertThrows(IllegalStateException.class, () -> selector.select(context));
    }

    record Risky() implements FlightAction {}
    static final class ScriptedMode implements FlightMode {
        final Vector3d velocity;
        final boolean wall;
        final boolean ground;
        Trajectory recovery;
        ScriptedMode(Vector3d velocity, boolean wall, boolean ground) {
            this.velocity = velocity; this.wall = wall; this.ground = ground;
        }
        @Override public List<? extends FlightAction> candidates(FlightContext c) { return List.of(new Risky()); }
        @Override public Trajectory predict(FlightContext c, FlightAction a) {
            return new Trajectory(c.state(), List.of(new PhysicsResult(c.state().position().add(0, -1, 0),
                    velocity, wall, false, ground, c.state().fallDistance())));
        }
        @Override public List<Trajectory> recover(FlightContext c, FlightAction a, Trajectory main) {
            return recovery == null ? List.of() : List.of(recovery);
        }
        @Override public void execute(FlightContext c, FlightAction a, MovementSync sync) {}
    }
}
