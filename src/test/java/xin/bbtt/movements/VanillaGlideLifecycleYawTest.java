package xin.bbtt.movements;

import org.junit.jupiter.api.Test;
import xin.bbtt.ElytraCommandTestSupport;

import static org.junit.jupiter.api.Assertions.assertEquals;

class VanillaGlideLifecycleYawTest extends ElytraCommandTestSupport {
    @Test
    void lowTakeoffKeepsNearestYawAcrossRepeatedLaunches() {
        movementSync.onGround.set(false);
        for (float current : new float[]{350f, -350f, 710f, -710f}) {
            movementSync.yaw.set(current);
            float expected = current > 0 ? current + 10f : current - 10f;
            for (int launch = 0; launch < 3; launch++) {
                var lifecycle = new VanillaGlideLifecycle();
                lifecycle.allowLowAltitudeTakeoff(0f, -15f);
                lifecycle.init(movementSync);
                assertEquals(expected, movementSync.yaw.get());
                assertEquals(-15f, movementSync.pitch.get());
            }
        }
    }

    @Test
    void lowTakeoffCrossesSignedDirectionBoundaryWithoutResettingTurns() {
        movementSync.onGround.set(false);
        movementSync.yaw.set(539f);
        var lifecycle = new VanillaGlideLifecycle();
        lifecycle.allowLowAltitudeTakeoff(-179f, -10f);
        lifecycle.init(movementSync);
        assertEquals(541f, movementSync.yaw.get());
        assertEquals(-10f, movementSync.pitch.get());
    }
}
