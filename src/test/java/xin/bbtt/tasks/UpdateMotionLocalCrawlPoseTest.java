package xin.bbtt.tasks;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.List;
import java.util.Map;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.movements.PathMovement;
import xin.bbtt.pathfinding.BuiltinMovementType;
import xin.bbtt.pathfinding.Node;
import xin.bbtt.pathfinding.PathStep;
import xin.bbtt.physics.PlayerPose;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

/** Synthetic touchdown roofs exercise production travel, not a separate physics driver. */
final class UpdateMotionLocalCrawlPoseTest extends ElytraCommandTestSupport {
    private static final BlockState ROCK = new BlockState("minecraft:stone", 1, Map.of(), "block", 1, true, "STONE");
    private static final BlockState AIR = new BlockState("minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");

    @ParameterizedTest
    @ValueSource(doubles = {1.0, 1.4})
    void standingMetadataDoesNotExpandLocalTouchdownCrawlBody(double clearance) throws Exception {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) { return p.y < 56 ? ROCK : AIR; }
            @Override public boolean isBoxColliding(double x1, double y1, double z1, double x2, double y2, double z2) {
                return y1 < 56 || x1 < 166 && x2 > 164 && z1 < -36 && z2 > -39
                        && y2 > 56 + clearance && y1 < 59;
            }
        };
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
        movementSync.onRecoveryTeleport();
        movementSync.position.set(new Vector3d(164.8, 56, -37.25));
        movementSync.velocity.set(new Vector3d(0, -.0784, 0));
        movementSync.onGround.set(true);
        movementSync.setGliding(false);
        movementSync.getSelfMetadata().put("pose", "STANDING");
        movementSync.setActiveGoal(new Vector3i(168, 56, -38));

        updateMotionTask task = new updateMotionTask();
        Field localPose = updateMotionTask.class.getDeclaredField("pose");
        localPose.setAccessible(true);
        // Last local flight pose, immediately after the server clears fall-flying on touchdown.
        localPose.set(task, PlayerPose.GLIDING);
        Method tick = updateMotionTask.class.getDeclaredMethod("tick");
        tick.setAccessible(true);
        tick.invoke(task);
        assertEquals(PlayerPose.SWIMMING, localPose.get(task));
        assertTrue(movementSync.onGround.get());

        Method dimensions = updateMotionTask.class.getDeclaredMethod("getBodyDimensions");
        dimensions.setAccessible(true);
        Object body = dimensions.invoke(task);
        Method height = body.getClass().getDeclaredMethod("height");
        height.setAccessible(true);
        assertEquals(PlayerPose.SWIMMING.height(), (double) height.invoke(body));
        Method colliding = updateMotionTask.class.getDeclaredMethod("isPlayerBoxColliding", Vector3d.class);
        colliding.setAccessible(true);
        assertFalse((boolean) colliding.invoke(task, movementSync.position.get()));

        PathMovement movement = new PathMovement(List.of(
                new PathStep(new Node(164, 56, -38), BuiltinMovementType.WALK),
                new PathStep(new Node(168, 56, -38), BuiltinMovementType.WALK)));
        movement.init();
        boolean crawlProgress = false;
        for (int i = 0; i < 190 && !movement.isFinished(); i++) {
            Vector3d before = new Vector3d(movementSync.position.get());
            boolean crawling = localPose.get(task) == PlayerPose.SWIMMING;
            movement.onTick();
            tick.invoke(task);
            crawlProgress |= crawling && movementSync.position.get().distanceSquared(before) > 1.e-6;
            assertEquals("STANDING", movementSync.getSelfMetadata().get("pose"));
            assertEquals(56, movementSync.position.get().y, 1.e-6, "crawl must stay on the floor");
        }
        assertTrue(crawlProgress, "production motion ticks must physically advance under the roof");
        assertEquals(PlayerPose.STANDING, localPose.get(task));
        assertEquals(PlayerPose.STANDING.height(), (double) height.invoke(dimensions.invoke(task)));
        assertTrue(movement.isFinished());
        assertEquals(PathMovement.Completion.NORMAL, movement.completion());
        assertTrue(movementSync.position.get().distance(new Vector3d(168.5, 56, -37.5)) < .6);
    }
}
