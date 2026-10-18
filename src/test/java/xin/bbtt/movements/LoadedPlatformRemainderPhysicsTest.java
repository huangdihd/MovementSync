package xin.bbtt.movements;

import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerInput;
import xin.bbtt.physics.WalkingPhysics;
import xin.bbtt.world.World;

import java.lang.reflect.Field;
import java.util.Map;
import java.util.concurrent.atomic.AtomicLong;

import static org.junit.jupiter.api.Assertions.*;

/** Starts at the physical lower landing; all remainder planning and walking are real. */
final class LoadedPlatformRemainderPhysicsTest {
    private static final BlockState STONE = new BlockState(
            "minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState(
            "minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");
    private static final Vector3i GOAL = new Vector3i(0, 73, 0);

    @AfterEach void resetSingleton() { MovementSync.INSTANCE = null; }

    private static World platform(boolean stairs) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                int x = (int) Math.floor(p.x), y = (int) Math.floor(p.y), z = (int) Math.floor(p.z);
                boolean top = x >= 0 && x <= 5 && z >= -1 && z <= 1 && y == 72;
                boolean stair = stairs && x >= 3 && x <= 5 && z >= -9 && z <= -2
                        && y >= 64 && y < 64 + z + 10;
                return y < 64 || top || stair ? STONE : AIR;
            }
        };
    }

    @Test void lowerLandingWalksStairsToFullThreeDimensionalGoalThenClearsRequest() throws Exception {
        exercise(true);
    }

    @Test void inaccessibleLoadedPlatformFailsInsteadOfArrivingBelowIt() throws Exception {
        exercise(false);
    }

    private static void exercise(boolean stairs) throws Exception {
        World world = platform(stairs);
        MovementSync sync = new MovementSync();
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(sync, world);
        sync.position.set(new Vector3d(.5, 64, .5));
        sync.velocity.set(new Vector3d());
        sync.onGround.set(true);
        sync.setFollowTargetId(-1);
        long generation = sync.beginGotoNavigationRequest(GOAL, true, false);
        TravelMovement travel = new TravelMovement(GOAL, generation, false);
        Field childField = TravelMovement.class.getDeclaredField("child");
        childField.setAccessible(true);
        Field watchdog = PathMovement.class.getDeclaredField("progressWatchdog");
        watchdog.setAccessible(true);
        AtomicLong clock = new AtomicLong();
        PathMovement walking = null;
        boolean climbed = false;
        for (int tick = 0; tick < 2400 && !travel.isFinished(); tick++) {
            if (sync.position.get().y < 72) {
                assertEquals(GOAL, sync.getActiveGoal(), "horizontal proximity below the platform is not arrival");
            }
            clock.addAndGet(50_000_000L);
            travel.onTick();
            if (childField.get(travel) instanceof PathMovement path && walking != path) {
                walking = path;
                watchdog.set(path, new PathProgressWatchdog(200, clock::get));
            }
            if (travel.isFinished()) break;
            var next = new WalkingPhysics(sync.getInput(), sync.yaw.get()).step(world,
                    new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get()),
                    (double) .6f / 2, 1.8);
            sync.position.set(next.position());
            sync.velocity.set(next.velocity());
            sync.onGround.set(next.onGround());
            climbed |= next.position().y > 65;
        }
        assertTrue(travel.isFinished(), "position=" + sync.position.get());
        assertNull(sync.getActiveGoal());
        assertEquals(PlayerInput.NONE, sync.getInput());
        if (stairs) {
            assertNotNull(walking, "reachable stairs must execute a real ground remainder");
            assertTrue(climbed);
            assertEquals(PathMovement.Completion.NORMAL, walking.completion());
            assertNull(walking.failureReason());
            assertTrue(sync.onGround.get());
            assertTrue(Math.abs(sync.position.get().x - .5) < .5);
            assertTrue(Math.abs(sync.position.get().z - .5) < .5);
            assertEquals(73, sync.position.get().y, .01);
        } else {
            // With no route at all, Travel rejects the remainder immediately;
            // the partial-route watchdog is covered by PathMovementPartialOutcomeTest.
            assertNull(walking);
            assertNull(sync.getGotoRecovery().getTask());
            assertEquals(64, sync.position.get().y, .01);
        }
    }
}
