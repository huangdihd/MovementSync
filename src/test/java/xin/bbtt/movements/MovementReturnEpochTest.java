package xin.bbtt.movements;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.KnownLandingReserve;

import static org.junit.jupiter.api.Assertions.*;

/** Movement-owned copies must not outlive the control epoch or invent resource shortages. */
final class MovementReturnEpochTest {
    private static Object get(ElytraFlyMovement movement, String name) throws Exception {
        var field = ElytraFlyMovement.class.getDeclaredField(name);
        field.setAccessible(true);
        return field.get(movement);
    }

    private static void set(ElytraFlyMovement movement, String name, Object value) throws Exception {
        var field = ElytraFlyMovement.class.getDeclaredField(name);
        field.setAccessible(true);
        field.set(movement, value);
    }

    private static boolean install(ElytraFlyMovement movement, KnownLandingReserve.Result reserve) throws Exception {
        var method = ElytraFlyMovement.class.getDeclaredMethod("installResourceReturn", KnownLandingReserve.Result.class);
        method.setAccessible(true);
        return (boolean) method.invoke(movement, reserve);
    }

    private static void refresh(ElytraFlyMovement movement, MovementSync sync) throws Exception {
        var method = ElytraFlyMovement.class.getDeclaredMethod("refreshReturnControlEpoch", MovementSync.class);
        method.setAccessible(true);
        method.invoke(movement, sync);
    }

    @Test void unknownWithSiteVetoesExposureButNeverInstallsReturn() throws Exception {
        MovementSync sync = new MovementSync();
        ElytraFlyMovement movement = new ElytraFlyMovement(new Vector3d(600, 40, 0));
        refresh(movement, sync);
        for (var status : KnownLandingReserve.Status.values()) {
            if (status == KnownLandingReserve.Status.VALID) continue;
            var unknown = new KnownLandingReserve.Result(status, new Vector3d(230, 34, 21),
                    -1, 1, Integer.MIN_VALUE, true, true);
            assertFalse(unknown.canContinue(), status.toString());
            assertFalse(install(movement, unknown), status.toString());
            assertNull(get(movement, "reserveReturnSite"));
            assertFalse((boolean) get(movement, "returnEpisodeSubmitted"));
        }
    }

    @Test void onlyValidThresholdInstallsResourceReturn() throws Exception {
        new MovementSync();
        ElytraFlyMovement movement = new ElytraFlyMovement(new Vector3d(600, 40, 0));
        Vector3d site = new Vector3d(230, 34, 21);
        assertFalse(install(movement, new KnownLandingReserve.Result(KnownLandingReserve.Status.VALID,
                site, 3, 1, 60, true, true)));
        assertNull(get(movement, "reserveReturnSite"));
        assertTrue(install(movement, new KnownLandingReserve.Result(KnownLandingReserve.Status.VALID,
                site, 3, 1, 0, true, true)));
        assertEquals(site, get(movement, "reserveReturnSite"));
    }

    @Test void changedEpochClearsInstalledReturnAndAllowsFreshAnchorImmediately() throws Exception {
        MovementSync sync = new MovementSync();
        ElytraFlyMovement movement = new ElytraFlyMovement(new Vector3d(600, 40, 0));
        refresh(movement, sync);
        Vector3d oldSite = new Vector3d(230, 34, 21);
        assertTrue(install(movement, new KnownLandingReserve.Result(KnownLandingReserve.Status.VALID,
                oldSite, 3, 1, -1, true, true)));
        set(movement, "returnEpisodeSubmitted", true);
        set(movement, "lastReturnPreviewTick", 500L);
        set(movement, "completedPlannedReturn", true);
        refresh(movement, sync);
        assertEquals(oldSite, get(movement, "reserveReturnSite"), "same epoch retains genuine return intent");
        assertTrue((boolean) get(movement, "returnEpisodeSubmitted"));
        sync.position.set(new Vector3d(-3.5, 36, -1.5));
        sync.invalidateFlightControls();
        refresh(movement, sync);
        assertNull(get(movement, "reserveReturnSite"), "correction must not steer back to A");
        assertFalse((boolean) get(movement, "returnEpisodeSubmitted"));
        assertFalse(movement.completedPlannedReturn());
        assertEquals(Long.MIN_VALUE, get(movement, "lastReturnPreviewTick"));
        assertNull(get(movement, "returnFailure"));
        Vector3d newSite = new Vector3d(4.5, 35, -1.5);
        assertTrue(install(movement, new KnownLandingReserve.Result(KnownLandingReserve.Status.VALID,
                newSite, 3, 1, 0, true, true)));
        assertEquals(newSite, get(movement, "reserveReturnSite"));
        assertEquals(new Vector3d(600, 40, 0), movement.getTarget(), "correction does not replace navigation destination");
    }
}
