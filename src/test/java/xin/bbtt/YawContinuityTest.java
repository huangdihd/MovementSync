package xin.bbtt;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

final class YawContinuityTest {
    @Test
    void directionsPreserveAccumulatedTurns() {
        assertEquals(360f, MovementSync.nearestEquivalentYaw(350f, 0f));
        assertEquals(-360f, MovementSync.nearestEquivalentYaw(-350f, 0f));
        assertEquals(-360f, MovementSync.nearestEquivalentYaw(-190f, 0f));
        assertEquals(180f, MovementSync.nearestEquivalentYaw(179f, -180f));
        assertEquals(720f, MovementSync.nearestEquivalentYaw(720f, 0f));
    }
}
