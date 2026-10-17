package xin.bbtt.recovery;

import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.commands.GotoCommandExecutor;
import xin.bbtt.listeners.MovementParametersPacketListener;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.player.ClientboundSetHealthPacket;
import java.util.concurrent.atomic.AtomicLong;
import static org.junit.jupiter.api.Assertions.*;

class StaticGotoRecoveryTest {
    @Test void requiresGameSameDimensionAndFirstTeleportAndClaimsOnce() {
        var recovery = new StaticGotoRecovery();
        recovery.remember(new Vector3i(1, 2, 3), true, "minecraft:overworld");
        recovery.suspend();
        assertNull(recovery.claim(true, "minecraft:overworld"));
        recovery.onFirstTeleport();
        assertNull(recovery.claim(false, "minecraft:overworld"));
        assertNull(recovery.claim(false, "minecraft:the_nether"));
        for (int i = 0; i < 20; i++) recovery.tick();
        var task = recovery.claim(true, "minecraft:overworld");
        assertNotNull(task);
        assertTrue(task.walk());
        assertEquals(new Vector3i(1, 2, 3), task.goal());
        task.goal().set(9, 9, 9);
        assertEquals(new Vector3i(1, 2, 3), task.goal());
        assertNull(recovery.claim(true, "minecraft:overworld"));
    }

    @Test void expiresAtThirtyMinutesWithoutRenewingOnReconnect() {
        AtomicLong now = new AtomicLong(100);
        var recovery = new StaticGotoRecovery(now::get);
        recovery.remember(new Vector3i(), false, "world");
        now.addAndGet(StaticGotoRecovery.MAX_AGE_MILLIS - 1);
        recovery.suspend();
        recovery.onFirstTeleport();
        assertFalse(recovery.expire());
        now.addAndGet(StaticGotoRecovery.MAX_AGE_MILLIS - 1);
        recovery.suspend();
        recovery.onFirstTeleport();
        assertFalse(recovery.expire());
        now.incrementAndGet();
        assertTrue(recovery.expire());
        assertNull(recovery.claim(true, "world"));
        assertNull(recovery.getTask());
    }

    @Test void disableAndSessionResetPreserveIntentButInvalidateOldRequests() {
        var sync = new MovementSync();
        sync.setRecoveryDimension("world");
        long old = sync.beginGotoNavigationRequest(new Vector3i(1, 2, 3), true, false);
        var task = sync.getGotoRecovery().getTask();
        sync.onDisable();
        sync.suspendNavigationForRecovery();
        assertSame(task, sync.getGotoRecovery().getTask());
        assertTrue(sync.getGotoRecovery().isPending());
        assertFalse(sync.isNavigationRequestCurrent(old));
        assertNull(sync.getActiveGoal());
        sync.cancelNavigation();
        assertNull(sync.getGotoRecovery().getTask());
    }

    @Test void arrivalReplacementAndHealthDeathClearIntent() {
        var sync = new MovementSync();
        Vector3i goal = new Vector3i(1, 2, 3);
        sync.beginGotoNavigationRequest(goal, false, false);
        sync.setActiveGoal(null);
        assertNull(sync.getGotoRecovery().getTask());
        sync.beginGotoNavigationRequest(goal, false, false);
        sync.beginNavigationRequest();
        assertNull(sync.getGotoRecovery().getTask());
        sync.beginGotoNavigationRequest(goal, false, false);
        new MovementParametersPacketListener().packetReceived(null, new ClientboundSetHealthPacket(0, 20, 5));
        assertNull(sync.getGotoRecovery().getTask());
    }

    @Test void onlyFirstPositionEnablesOneShotAirRecoveryUntilSessionReset() throws Exception {
        var sync = new MovementSync();
        var checks = new java.util.concurrent.atomic.AtomicInteger();
        var world = new xin.bbtt.world.World() {
            @Override public boolean chunkLoaded(int x, int z) {
                checks.incrementAndGet();
                return true;
            }
        };
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(sync, world);
        sync.onGround.set(true);
        sync.suspendNavigationForRecovery();
        var air = sync.getReconnectAirRecovery();
        air.tick(sync, true, false);
        assertEquals(0, checks.get(), "No aerial recovery before authoritative position");
        sync.onRecoveryTeleport();
        air.tick(sync, true, false);
        assertEquals(1, checks.get(), "First position enables aerial safety check");
        for (int i = 0; i < 3; i++) {
            sync.onRecoveryTeleport();
            air.tick(sync, true, false);
        }
        assertEquals(1, checks.get(), "Later teleports must not restart consumed aerial recovery");
        sync.suspendNavigationForRecovery();
        air.tick(sync, true, false);
        assertEquals(1, checks.get());
        sync.onRecoveryTeleport();
        air.tick(sync, true, false);
        assertEquals(2, checks.get(), "A new session enables one new safety check");
    }

    @Test void everyTeleportRestartsTwentyQuietTicksAndClaimIsOneShot() {
        var recovery = new StaticGotoRecovery();
        recovery.remember(new Vector3i(1, 2, 3), true, "world");
        var task = recovery.getTask();
        recovery.suspend();
        assertTrue(recovery.onTeleport());
        for (int correction = 0; correction < 3; correction++) {
            for (int i = 0; i < 19; i++) {
                recovery.tick();
                assertNull(recovery.claim(true, "world"));
            }
            assertFalse(recovery.onTeleport(), "Only initial positioning is first");
            assertNull(recovery.claim(true, "world"));
        }
        for (int i = 0; i < 19; i++) {
            recovery.tick();
            assertNull(recovery.claim(true, "world"));
        }
        recovery.tick();
        assertSame(task, recovery.claim(true, "world"));
        assertFalse(recovery.isPending());
        recovery.onTeleport();
        for (int i = 0; i < 40; i++) {
            recovery.tick();
            assertNull(recovery.claim(true, "world"));
        }
    }

    @Test void pendingCorrectionsDelayResumeAndOrdinaryCorrectionAfterResumeCancelsTask() {
        var sync = new MovementSync();
        sync.setRecoveryDimension("world");
        sync.beginGotoNavigationRequest(new Vector3i(4, 0, 0), true, false);
        var task = sync.getGotoRecovery().getTask();
        sync.suspendNavigationForRecovery();
        long suspended = sync.getNavigationGeneration();
        sync.onRecoveryTeleport();
        for (int i = 0; i < 19; i++) sync.tickNavigationRecovery(true);
        sync.onRecoveryTeleport();
        assertTrue(sync.getGotoRecovery().isPending());
        assertSame(task, sync.getGotoRecovery().getTask());
        for (int i = 0; i < 19; i++) sync.tickNavigationRecovery(true);
        assertEquals(suspended, sync.getNavigationGeneration());
        sync.tickNavigationRecovery(true);
        long resumed = sync.getNavigationGeneration();
        assertEquals(suspended + 1, resumed);
        assertFalse(sync.getGotoRecovery().isPending());
        assertSame(task, sync.getGotoRecovery().getTask());
        sync.onRecoveryTeleport();
        assertEquals(resumed + 1, sync.getNavigationGeneration());
        assertFalse(sync.isNavigationRequestCurrent(resumed));
        assertNull(sync.getGotoRecovery().getTask());
        assertNull(sync.getActiveGoal());
        assertNull(sync.getMovementController().getCurrentMovement());
        for (int i = 0; i < 40; i++) sync.tickNavigationRecovery(true);
        assertEquals(resumed + 1, sync.getNavigationGeneration());
    }

    @Test void longRunningTaskDoesNotExpireAndRepeatedSessionResetsDoNotRenewSuspension() {
        AtomicLong now = new AtomicLong(100);
        var recovery = new StaticGotoRecovery(now::get);
        recovery.remember(new Vector3i(1, 2, 3), false, "world");
        var task = recovery.getTask();
        now.addAndGet(StaticGotoRecovery.MAX_AGE_MILLIS * 4);
        assertFalse(recovery.expire(), "Running time is not suspension time");
        assertSame(task, recovery.getTask());
        recovery.suspend();
        long suspendedAt = now.get();
        for (int i = 1; i <= 3; i++) {
            now.set(suspendedAt + i * StaticGotoRecovery.MAX_AGE_MILLIS / 4);
            recovery.onTeleport();
            recovery.tick();
            recovery.suspend();
            assertFalse(recovery.hasPosition());
            assertTrue(recovery.isPending());
            assertFalse(recovery.expire());
            assertSame(task, recovery.getTask());
        }
        now.set(suspendedAt + StaticGotoRecovery.MAX_AGE_MILLIS - 1);
        assertFalse(recovery.expire());
        now.incrementAndGet();
        assertTrue(recovery.expire(), "Expiry is measured from the first suspension");
        assertNull(recovery.getTask());
        assertFalse(recovery.isPending());
    }

    @Test void resumedTaskCanRunPastExpiryAndNextSuspensionGetsItsOwnClock() {
        AtomicLong now = new AtomicLong(100);
        var recovery = new StaticGotoRecovery(now::get);
        recovery.remember(new Vector3i(), true, "world");
        var task = recovery.getTask();
        recovery.suspend();
        recovery.onFirstTeleport();
        for (int i = 0; i < 20; i++) recovery.tick();
        assertSame(task, recovery.claim(true, "world"));
        now.addAndGet(StaticGotoRecovery.MAX_AGE_MILLIS * 4);
        assertFalse(recovery.expire());
        assertSame(task, recovery.getTask());
        recovery.suspend();
        assertFalse(recovery.expire());
        now.addAndGet(StaticGotoRecovery.MAX_AGE_MILLIS - 1);
        assertFalse(recovery.expire());
        now.incrementAndGet();
        assertTrue(recovery.expire());
        assertEquals(100, task.createdAt(), "Resuming must not renew task creation time");
    }

    @Test void resumesThroughOriginalGotoPathWithoutRenewingIntent() {
        var sync = new MovementSync();
        sync.setRecoveryDimension("world");
        new GotoCommandExecutor().onCommand(null, "goto", new String[] {"4", "0", "0", "walk"});
        var task = sync.getGotoRecovery().getTask();
        assertNotNull(task);
        long old = sync.getNavigationGeneration();
        sync.suspendNavigationForRecovery();
        sync.tickNavigationRecovery(true);
        assertEquals(old + 1, sync.getNavigationGeneration());
        sync.onRecoveryTeleport();
        for (int i = 0; i < 20; i++) sync.tickNavigationRecovery(true);
        assertEquals(old + 2, sync.getNavigationGeneration());
        assertSame(task, sync.getGotoRecovery().getTask());
        assertFalse(sync.getGotoRecovery().isPending());
        sync.tickNavigationRecovery(true);
        assertEquals(old + 2, sync.getNavigationGeneration());
    }
}
