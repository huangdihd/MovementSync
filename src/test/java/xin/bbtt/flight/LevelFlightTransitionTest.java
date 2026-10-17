package xin.bbtt.flight;

import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.physics.PhysicsResult;
import xin.bbtt.physics.PhysicsState;

import static org.junit.jupiter.api.Assertions.*;

final class LevelFlightTransitionTest {
    @Test void exampleTakeoffAndOptInLandingAreSafelySelectedAndExecuted() {
        var mode = new LevelFlightModeExample();
        mode.landingEnabled = true;
        var context = FlightExtensionContractTest.context();
        var config = new FlightContextBuilder().addMode(mode)
                .addDamageRule(BuiltinDamageRule.WALL).addDamageRule(BuiltinDamageRule.FALL).build();
        for (var kind : FlightTransitions.Kind.values()) {
            var selected = FlightTransitions.select(config, context, kind);
            assertNotNull(selected);
            assertSame(mode, selected.mode());
            assertEquals(0, mode.executions);
            selected.movement().init();
            selected.movement().onTick();
            assertTrue(selected.movement().isFinished());
        }
        assertEquals(1, mode.takeoffTicks);
        assertEquals(1, mode.landingTicks);
    }

    @Test void exampleWallAndFallTakeoffsRejectedEvenWithEmergencyAndDamageFirst() {
        for (boolean wall : new boolean[]{false, true}) {
            var mode = new LevelFlightModeExample();
            var base = FlightExtensionContractTest.context(FlightExtensionContractTest.LOADED,
                    new PhysicsState(new Vector3d(8, 100, 8),
                            wall ? new Vector3d(2, 0, 0) : new Vector3d(0, -1, 0), false, 12));
            var context = new FlightContext(base.world(), base.state(), base.movement(), base.damage(),
                    base.health(), base.target(), 0, 0, 0, false, 2, true, Map.of());
            mode.transitionStepper = (world, state, width, height) -> new PhysicsResult(
                    state.position().add(0, -1, 0), new Vector3d(), wall, false, !wall, 12);
            var config = new FlightContextBuilder().addMode(mode).cruiseDamageFirst(true)
                    .addDamageRule(BuiltinDamageRule.WALL).addDamageRule(BuiltinDamageRule.FALL)
                    .addCostTerm((c, a, t) -> -1e12).build();
            assertNull(FlightTransitions.select(config, context, FlightTransitions.Kind.TAKEOFF));
            assertEquals(0, mode.takeoffTicks);
        }
    }
}
