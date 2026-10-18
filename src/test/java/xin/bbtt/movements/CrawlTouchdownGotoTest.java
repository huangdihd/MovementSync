package xin.bbtt.movements;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Map;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.MovementSync;
import xin.bbtt.pathfinding.*;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Hypothetical low roofs at the reported coordinates, not captured server geometry. */
final class CrawlTouchdownGotoTest {
    private static final BlockState ROCK = new BlockState("minecraft:stone", 1, Map.of(), "block", 1, true, "STONE");
    private static final BlockState AIR = new BlockState("minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");
    @AfterEach void reset() { MovementSync.INSTANCE = null; }

    private World roof(double clearance) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) { return p.y < 56 ? ROCK : AIR; }
            @Override public boolean isBoxColliding(double x1, double y1, double z1, double x2, double y2, double z2) {
                return y1 < 56 || x1 < 166 && x2 > 164 && z1 < -36 && z2 > -39
                        && y2 > 56 + clearance && y1 < 59;
            }
        };
    }

    @Test void lowRoofTouchdownCrawlsThenCompletesStandingGoto() throws Exception {
        for (double clearance : new double[]{1.0, 1.4}) {
            World world = roof(clearance);
            MovementSync sync = new MovementSync();
            sync.onRecoveryTeleport();
            Field field = MovementSync.class.getDeclaredField("world");
            field.setAccessible(true);
            field.set(sync, world);
            sync.position.set(new Vector3d(164.8, 56, -37.25));
            sync.velocity.set(new Vector3d(0, -.0784, 0));
            sync.onGround.set(true);
            sync.setActiveGoal(new Vector3i(168, 56, -38));
            PlayerPose pose = PlayerPose.GLIDING.afterTravel(world, sync.position.get(), false, false, false);
            assertEquals(PlayerPose.SWIMMING, pose);
            PathMovement movement = new PathMovement(List.of(
                    new PathStep(new Node(164, 56, -38), BuiltinMovementType.WALK),
                    new PathStep(new Node(168, 56, -38), BuiltinMovementType.WALK)));
            movement.init();
            boolean crawled = false;
            for (int tick = 0; tick < 190 && !movement.isFinished(); tick++) {
                movement.onTick();
                var state = new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get(), 0, pose);
                var next = new WalkingPhysics(sync.getInput(), sync.yaw.get(), true, sync.getMovementParameters())
                        .step(world, state, PlayerPose.HALF_WIDTH, pose.height());
                crawled |= pose == PlayerPose.SWIMMING && next.position().distanceSquared(state.position()) > 1.e-6;
                sync.position.set(next.position());
                sync.velocity.set(next.velocity());
                sync.onGround.set(next.onGround());
                pose = next.pose();
            }
            assertTrue(crawled);
            assertEquals(PlayerPose.STANDING, pose);
            assertTrue(movement.isFinished());
            assertEquals(PathMovement.Completion.NORMAL, movement.completion());
            assertTrue(sync.position.get().distance(new Vector3d(168.5, 56, -37.5)) < .6);
            Field rejected = PathMovement.class.getDeclaredField("rejectedTransitions");
            rejected.setAccessible(true);
            assertTrue(((java.util.Set<?>) rejected.get(movement)).isEmpty());
        }
    }

    @Test void dryCrawlUsesFloatDimensionsAndSneakingSpeedWithoutSneakKey() {
        World world = roof(1.4);
        Vector3d p = new Vector3d(164.8, 56, -37.25);
        assertEquals((double) .6f, PlayerPose.SWIMMING.height());
        var input = new PlayerInput(true, false, false, false, false, false, true);
        var initial = new PhysicsState(p, new Vector3d(0, -.0784, 0), true, 0, PlayerPose.SWIMMING);
        var next = new WalkingPhysics(input, -90).step(world, initial, PlayerPose.HALF_WIDTH, initial.pose().height());
        assertTrue(next.position().x > p.x);
        assertTrue(next.position().x - p.x < .04, "dry crawl slows input and disallows sprint");
        assertTrue(DeploymentSafety.assessWalking(world, initial, MovementParameters.DEFAULT, -90, input).safe());
    }
}
