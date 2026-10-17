package xin.bbtt.flight;

import java.lang.reflect.Field;
import java.util.List;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

final class FlightRegistrationTest {
    @Test void globalRegistrationsAreCopiedAndCleanedUpWithoutPollutingOtherTests() throws Exception {
        synchronized (FlightContextBuilder.class) {
            var mode = new LevelFlightModeExample();
            FlightCostTerm cost = (c, a, t) -> 7;
            DamageRule damage = (c, a, before, after) -> new DamageRule.Damage(0, 0);
            var builder = new FlightContextBuilder().addStandard();
            var before = builder.build();
            try {
                FlightContextBuilder.registerGlobalMode(mode);
                FlightContextBuilder.registerGlobalCostTerm(cost);
                FlightContextBuilder.registerGlobalDamageRule(damage);
                var after = builder.build();
                assertEquals(before.modes().size() + 1, after.modes().size());
                assertEquals(before.costs().size() + 1, after.costs().size());
                assertEquals(before.damageRules().size() + 1, after.damageRules().size());
                assertFalse(before.modes().contains(mode));
                assertFalse(before.costs().contains(cost));
                assertFalse(before.damageRules().contains(damage));
                assertTrue(after.modes().contains(mode));
                assertTrue(after.costs().contains(cost));
                assertTrue(after.damageRules().contains(damage));
            } finally {
                removeGlobal("GLOBAL_MODES", mode);
                removeGlobal("GLOBAL_COSTS", cost);
                removeGlobal("GLOBAL_DAMAGE", damage);
            }
            assertEquals(before, builder.build());
        }
    }

    private static void removeGlobal(String name, Object value) throws Exception {
        Field field = FlightContextBuilder.class.getDeclaredField(name);
        field.setAccessible(true);
        ((List<?>) field.get(null)).remove(value);
    }

    @Test void navigationSnapshotsCannotBeChangedThroughBuilderOrLists() {
        var firstMode = new LevelFlightModeExample();
        var secondMode = new LevelFlightModeExample();
        var builder = new FlightContextBuilder().addMode(firstMode);
        var first = builder.build();
        var second = builder.addMode(secondMode).cruiseDamageFirst(true).build();
        assertFalse(first.modes().contains(secondMode));
        assertTrue(second.modes().contains(secondMode));
        assertFalse(first.cruiseDamageFirst());
        assertTrue(second.cruiseDamageFirst());
        assertThrows(UnsupportedOperationException.class, () -> first.modes().clear());
        assertThrows(UnsupportedOperationException.class, () -> first.costs().clear());
        assertThrows(UnsupportedOperationException.class, () -> first.damageRules().clear());
    }

    @Test void standardRegistrationIsIdempotentAndCruiseOptInChangesLegacyPolicy() {
        var builder = new FlightContextBuilder().addStandard();
        var once = builder.build();
        assertEquals(once, builder.addStandard().build());
        assertTrue(once.isLegacy());
        assertFalse(builder.cruiseDamageFirst(true).build().isLegacy());
        assertFalse(new FlightContextBuilder().addStandard().addMode(new LevelFlightModeExample()).build().isLegacy());
    }
}
