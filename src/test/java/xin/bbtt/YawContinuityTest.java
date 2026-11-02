package xin.bbtt;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

final class YawContinuityTest {
    @Test
    void directionsPreserveAccumulatedTurns() {
        assertEquals(360f, MovementSync.nearestEquivalentYaw(350f, 0f));
        assertEquals(-360f, MovementSync.nearestEquivalentYaw(-350f, 0f));
        assertEquals(-360f, MovementSync.nearestEquivalentYaw(-190f, 0f));
        assertEquals(180f, MovementSync.nearestEquivalentYaw(179f, -180f));
        assertEquals(720f, MovementSync.nearestEquivalentYaw(720f, 0f));
    }

    @Test
    void extremeAccumulatedYawNeverRoundsPastHalfTurn() {
        assertBoundedTurn(268_435_648f, 28f);
        assertBoundedTurn(-268_435_648f, -28f);
        assertBoundedTurn(268_435_648f, 24f);
        assertBoundedTurn(-268_435_648f, -24f);
        assertBoundedTurn(Float.MAX_VALUE, 0f);
        assertBoundedTurn(-Float.MAX_VALUE, 0f);
    }

    private static void assertBoundedTurn(float current, float direction) {
        float result = MovementSync.nearestEquivalentYaw(current, direction);
        assertTrue(Float.isFinite(result));
        assertTrue(Math.abs((double) result - current) <= 180,
                () -> "turn exceeded a half rotation: " + current + " -> " + result);
    }
}
