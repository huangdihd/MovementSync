package xin.bbtt.movements;

import java.util.Map;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.*;
import xin.bbtt.tasks.updateMotionTask;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

class CorridorStopLandingParityTest extends ElytraCommandTestSupport {
    @Test void stopInsideTunnelUsesReachableLandingAndVanillaBodyForEveryTravelStep() throws Exception {
        movementSync.getMovementController().pause();
        var stone = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
        var air = new BlockState("minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                return p.z < 0 || p.z >= 9 || p.y < 61 || p.y >= 76 ? stone : air;
            }
        };
        var wf = MovementSync.class.getDeclaredField("world"); wf.setAccessible(true); wf.set(movementSync, world);
        movementSync.position.set(new Vector3d(10.5, 67, 4.5));
        movementSync.velocity.set(new Vector3d(.6, -.1, 0));
        movementSync.yaw.set(-90f);
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        movementSync.cancelNavigation();
        var sf = MovementSync.class.getDeclaredField("safetyLanding"); sf.setAccessible(true);
        var landing = assertInstanceOf(ElytraFlyMovement.class, sf.get(movementSync));
        landing.init();
        var dimensions = updateMotionTask.class.getDeclaredMethod("getBodyDimensions"); dimensions.setAccessible(true);
        Object body = dimensions.invoke(new updateMotionTask());
        var width = body.getClass().getDeclaredMethod("halfWidth"); width.setAccessible(true);
        var height = body.getClass().getDeclaredMethod("height"); height.setAccessible(true);
        double hw = (double) width.invoke(body), h = (double) height.invoke(body);
        try {
            for (int tick = 0; tick < 200 && !movementSync.onGround.get(); tick++) {
                var pf = ElytraFlyMovement.class.getDeclaredField("lowPlan"); pf.setAccessible(true);
                if (pf.get(landing) instanceof CompletableFuture<?> pending) pending.get(20, TimeUnit.SECONDS);
                landing.onTick();
                var state = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(), false,
                        movementSync.fallDistance.get(), PlayerPose.GLIDING);
                var physics = new GlidingPhysics(movementSync.yaw.get(), movementSync.pitch.get(), false,
                        movementSync.snapshotInput(), movementSync.getMovementParameters());
                var live = physics.step(world, state, hw, h);
                var vanillaBody = physics.step(world, state, (double) (.6f / 2), (double) .6f);
                assertEquals(vanillaBody.position(), live.position());
                assertEquals(vanillaBody.velocity(), live.velocity());
                assertTrue(live.position().y >= 61 && live.position().y + h <= 76);
                assertFalse(live.collidedX() || live.collidedZ());
                movementSync.position.set(live.position()); movementSync.velocity.set(live.velocity());
                movementSync.onGround.set(live.onGround()); movementSync.fallDistance.set(live.fallDistance());
            }
            assertTrue(movementSync.onGround.get(), "stop must finish inside the tunnel");
            assertEquals(61, movementSync.position.get().y);
            landing.onTick();
            assertTrue(landing.isFinished());
        } finally { landing.onStop(); }
    }
}
