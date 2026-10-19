package xin.bbtt.physics;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

final class FireworkScheduleTest {
    @Test void scheduledReachRejectsLavaBeforeDelayedRecovery() {
        var air = new xin.bbtt.world.World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
        };
        var lava = new xin.bbtt.world.World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public xin.bbtt.Block.BlockState getBlockStateAt(org.joml.Vector3d p) {
                if (p.y < 32) return new xin.bbtt.Block.BlockState("minecraft:lava", 0,
                        java.util.Map.of(), "empty", 0, false, "");
                return super.getBlockStateAt(p);
            }
        };
        var initial = new PhysicsState(new org.joml.Vector3d(8, 32.5, 8),
                new org.joml.Vector3d(.8, -.3, 0), false, 0, PlayerPose.GLIDING);
        var target = new org.joml.Vector3d(60, 32, 8);
        assertTrue(GlideRange.scheduledReach(air, initial, MovementParameters.DEFAULT,
                target, new int[]{1, 1, 1}, 3, 15, 0, 2));
        assertFalse(GlideRange.scheduledReach(lava, initial, MovementParameters.DEFAULT,
                target, new int[]{1, 1, 1}, 3, 15, 0, 2));
        var high = new PhysicsState(new org.joml.Vector3d(8, 50, 8), initial.velocity(), false);
        assertTrue(GlideRange.scheduledReach(lava, high, MovementParameters.DEFAULT,
                target, new int[]{1, 1, 1}, 3, 0, 0, 2));
        assertFalse(GlideRange.scheduledReach(new xin.bbtt.world.World(), high,
                MovementParameters.DEFAULT, target, new int[]{1}, 1, 0, 0, 2));
    }

    @Test void consecutiveCreditsExactlyTheRequestedTicks() {
        var schedule = FireworkSchedule.consecutive(63);
        for (int tick = 0; tick < 63; tick++) assertTrue(schedule.nextPowered());
        assertTrue(schedule.exhausted());
        assertFalse(schedule.nextPowered());
    }

    @Test void confirmationConsumesRocketAgeAndCooldownSeparatesUses() {
        var schedule = FireworkSchedule.confirmed(new int[]{1, 0}, 2, 3, 1, 2);
        for (int tick = 0; tick < 56; tick++) {
            boolean expected = tick == 0 || tick >= 5 && tick <= 23 || tick >= 45 && tick <= 53;
            assertEquals(expected, schedule.nextPowered(), "tick " + tick);
        }
        assertTrue(schedule.exhausted());
    }

    @Test void expiredBeforeConfirmationCannotCreateBoost() {
        var schedule = FireworkSchedule.confirmed(new int[]{0}, 1, 0, 0, 12);
        for (int tick = 0; tick < 20; tick++) assertFalse(schedule.nextPowered());
        assertTrue(schedule.exhausted());
    }
}
