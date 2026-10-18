package xin.bbtt.movements;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.Server;
import xin.bbtt.tasks.updateMotionTask;

import static org.junit.jupiter.api.Assertions.*;

class CompletedPhysicsClockTest extends PhysicalClockTestSupport {
    @Test
    void repeatedInputSnapshotsDoNotAdvanceCompletedPhysicsClock() {
        long initial = movementSync.physicsTickVersion();
        var position = new Vector3d(movementSync.position.get());
        var velocity = new Vector3d(movementSync.velocity.get());
        for (int i = 0; i < 100; i++) movementSync.snapshotInput();
        assertEquals(initial, movementSync.physicsTickVersion());
        assertEquals(position, movementSync.position.get());
        assertEquals(velocity, movementSync.velocity.get());
        physicalTick();
        assertEquals(initial + 1, movementSync.physicsTickVersion());
        for (int i = 0; i < 100; i++) movementSync.snapshotInput();
        assertEquals(initial + 1, movementSync.physicsTickVersion());
    }

    @Test
    void actualMotionTaskAdvancesClockOncePerCompletedStep() throws Exception {
        var running = Bot.class.getDeclaredField("running");
        running.setAccessible(true);
        var previousRunning = running.get(Bot.INSTANCE);
        var previousServer = Bot.INSTANCE.getServer();
        try {
            movementSync.onRecoveryTeleport();
            installAirborneWorld();
            movementSync.setGliding(true);
            running.set(Bot.INSTANCE, true);
            Bot.INSTANCE.setServer(Server.Game);
            long initial = movementSync.physicsTickVersion();
            double initialY = movementSync.position.get().y;
            var task = new updateMotionTask();
            task.run();
            assertEquals(initial + 1, movementSync.physicsTickVersion());
            assertTrue(movementSync.position.get().y < initialY);
            for (int i = 0; i < 100; i++) movementSync.snapshotInput();
            assertEquals(initial + 1, movementSync.physicsTickVersion());
            double firstY = movementSync.position.get().y;
            task.run();
            assertEquals(initial + 2, movementSync.physicsTickVersion());
            assertTrue(movementSync.position.get().y < firstY);
        } finally {
            running.set(Bot.INSTANCE, previousRunning);
            Bot.INSTANCE.setServer(previousServer);
        }
    }
}
