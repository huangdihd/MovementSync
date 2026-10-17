package xin.bbtt.flight;

import java.util.List;
import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class FlightHazardRulesTest {
    private static World world(String name, boolean lit, int hx, int hy, int hz) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                return new BlockState((int)Math.floor(p.x) == hx && (int)Math.floor(p.y) == hy
                        && (int)Math.floor(p.z) == hz ? name : "air", 0,
                        Map.of("lit", Boolean.toString(lit)), "empty", 0, false, "");
            }
        };
    }

    private static FlightContext context(World world, Vector3d position) {
        return FlightExtensionContractTest.context(world, new PhysicsState(position, new Vector3d(), false));
    }

    @Test void sweptHazardsApplyToNonGlideActionsAndCannotTunnel() {
        FlightAction action = new FlightTransitionsTest.Action(0);
        for (String block : List.of("lava", "minecraft:lava_cauldron", "fire", "soul_fire",
                "campfire", "soul_campfire", "magma_block")) {
            var c = context(world(block, true, 9, 100, 8), new Vector3d(8, 100, 8.5));
            var end = new PhysicsResult(new Vector3d(11, 100, 8.5), new Vector3d(), false, false);
            var rule = block.contains("lava") ? BuiltinDamageRule.LAVA : BuiltinDamageRule.FIRE;
            assertEquals(block.contains("lava") ? 4 : block.startsWith("soul") ? 2 : 1,
                    rule.damage(c, action, c.state(), end).raw(), block);
            assertFalse(FlightHazards.safeLanding(c.world(), new Vector3d(9.5, 101, 8.5)), block);
        }
    }

    @Test void unlitCampfiresAndOffPathDiagonalCellsAreSafe() {
        var c = context(world("campfire", false, 9, 100, 8), new Vector3d(8, 100, 8.5));
        var end = new PhysicsResult(new Vector3d(11, 100, 8.5), new Vector3d(), false, false);
        assertEquals(0, BuiltinDamageRule.FIRE.damage(c, new FlightTransitionsTest.Action(0), c.state(), end).raw());
        c = context(world("lava", true, 8, 100, 11), new Vector3d(8, 100, 8));
        end = new PhysicsResult(new Vector3d(12, 100, 12), new Vector3d(), false, false);
        assertEquals(0, BuiltinDamageRule.LAVA.damage(c, new FlightTransitionsTest.Action(0), c.state(), end).raw());
    }

    @Test void allModesTransitionsUseStandardRulesBeforeFactories() {
        for (var kind : FlightTransitions.Kind.values()) {
            var first = new FlightTransitionsTest.Mode(0);
            var second = new FlightTransitionsTest.Mode(1);
            var c = context(world("lava", true, 8, 100, 8), new Vector3d(8.5, 100, 8.5));
            var builder = new FlightContextBuilder().addMode(first).addMode(second);
            for (var rule : BuiltinDamageRule.values()) builder.addDamageRule(rule);
            assertNull(FlightTransitions.select(builder.build(), c, kind));
            assertEquals(0, first.factories);
            assertEquals(0, second.factories);
            builder.cruiseDamageFirst(true);
            var selected = FlightTransitions.select(builder.build(), c, kind);
            assertNull(selected, "Hazards remain forbidden during damage-first landing");
        }
    }

    @Test void standardRegistersBothEnvironmentalRules() {
        var rules = new FlightContextBuilder().addStandard().addStandard().build().damageRules();
        assertEquals(1, rules.stream().filter(r -> r == BuiltinDamageRule.LAVA).count());
        assertEquals(1, rules.stream().filter(r -> r == BuiltinDamageRule.FIRE).count());
    }
}
