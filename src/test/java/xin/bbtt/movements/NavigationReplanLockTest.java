package xin.bbtt.movements;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.MovementSync;
import xin.bbtt.pathfinding.BuiltinMovementType;
import xin.bbtt.pathfinding.Node;
import xin.bbtt.pathfinding.PathStep;
import xin.bbtt.world.World;

final class NavigationReplanLockTest {
    private static final BlockState STONE = new BlockState(
        "minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState(
        "minecraft:air", 0, Map.of(), "empty", 0.0, false, "AIR");

    @AfterEach
    void resetSingleton() {
        MovementSync.INSTANCE = null;
    }

    @Test
    void cancellationIsNotBlockedByAStalledReplan() throws Exception {
        CountDownLatch plannerEntered = new CountDownLatch(1);
        CountDownLatch releasePlanner = new CountDownLatch(1);
        MovementSync movementSync = new MovementSync();
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                plannerEntered.countDown();
                try {
                    releasePlanner.await(2, TimeUnit.SECONDS);
                } catch (InterruptedException error) {
                    Thread.currentThread().interrupt();
                }
                return true;
            }

            @Override
            public BlockState getBlockStateAt(Vector3d position) {
                return Math.floor(position.y) == 63 ? STONE : AIR;
            }
        });
        movementSync.position.set(new Vector3d(0.5, 64.0, 0.5));
        movementSync.onGround.set(true);
        Vector3i goal = new Vector3i(8, 64, 0);
        long generation = movementSync.beginStaticNavigationRequest(goal);
        PathMovement movement = new PathMovement(
            List.of(
                new PathStep(new Node(0, 64, 0), BuiltinMovementType.WALK),
                new PathStep(new Node(8, 64, 0), BuiltinMovementType.WALK)
            ),
            -1, false, goal, -1, generation
        );
        movement.requestRepath();
        CompletableFuture<Void> planner = CompletableFuture.runAsync(movement::onTick);
        assertTrue(plannerEntered.await(1, TimeUnit.SECONDS));

        CompletableFuture<Void> cancellation = CompletableFuture.runAsync(movementSync::cancelNavigation);
        boolean cancelledPromptly;
        try {
            cancellation.get(500, TimeUnit.MILLISECONDS);
            cancelledPromptly = true;
        } catch (TimeoutException timeout) {
            cancelledPromptly = false;
        } finally {
            releasePlanner.countDown();
        }
        planner.get(2, TimeUnit.SECONDS);
        cancellation.get(2, TimeUnit.SECONDS);

        assertTrue(cancelledPromptly,
            "cancelNavigation must not wait for D* replan computation");
    }
}
