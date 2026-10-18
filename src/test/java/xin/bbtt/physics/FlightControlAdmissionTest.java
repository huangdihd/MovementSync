package xin.bbtt.physics;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

class FlightControlAdmissionTest extends ElytraCommandTestSupport {
    private void world() throws Exception {
        var field = xin.bbtt.MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public xin.bbtt.Block.BlockState getBlockStateAt(Vector3d p) {
                return p.y < 0 ? new xin.bbtt.Block.BlockState("minecraft:stone", 1, java.util.Map.of(),
                        "block", 1.5, true, "STONE") : super.getBlockStateAt(p);
            }
            @Override public boolean isBoxColliding(double x, double y, double z, double X, double Y, double Z) {
                return y < 0 || X > 3;
            }
        });
        movementSync.position.set(new Vector3d(0, 20, 0));
        movementSync.velocity.set(new Vector3d(0, 0, .3));
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
    }
    private FlightControlAdmission.Envelope envelope(Object owner, AtomicInteger accepted) {
        var source = FlightControlAdmission.snapshot(movementSync);
        var decision = new ElytraFlightController.Decision(0, 0, true, ElytraFlightController.Phase.CRUISE);
        var route = new WaypointFlightTarget(List.of(new Vector3d(0,20,0), new Vector3d(0,20,100)),
                new Vector3d(0,20,100), false, false, 3);
        return FlightControlAdmission.prepare(movementSync, owner, source, decision, route, accepted::incrementAndGet);
    }
    private void physicalTick() {
        var input = movementSync.snapshotInput();
        var state = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(), false);
        var result = new GlidingPhysics(movementSync.yaw.get(), movementSync.pitch.get(), false, input,
                movementSync.getMovementParameters()).step(movementSync.getWorld(), state,
                PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
        assertFalse(result.collidedX());
        assertFalse(result.collidedZ());
        movementSync.position.set(result.position());
        movementSync.velocity.set(result.velocity());
    }
    @Test void delayedSelectedControlIsRevalidatedAtPhysicalBoundary() throws Exception {
        for (int delay = 3; delay <= 8; delay++) {
            world();
            var accepted = new AtomicInteger();
            var envelope = envelope(new Object(), accepted);
            for (int i = 0; i < delay; i++) physicalTick();
            movementSync.submitFlightControls(envelope);
            physicalTick();
            assertEquals(1, accepted.get(), "delay=" + delay);
            assertTrue(Math.abs(movementSync.position.get().x) < 3);
        }
    }
    @Test void cancelledOwnerCannotQueueRocket() throws Exception {
        world();
        Object owner = new Object();
        var accepted = new AtomicInteger();
        movementSync.submitFlightControls(envelope(owner, accepted));
        movementSync.revokeFlightControls(owner);
        physicalTick();
        assertEquals(0, accepted.get());
        assertNull(movementSync.pollBeforeMovementPacket());
    }
    @Test void displacedStateRejectsRocketAndControls() throws Exception {
        world();
        var accepted = new AtomicInteger();
        movementSync.submitFlightControls(envelope(new Object(), accepted));
        movementSync.position.set(new Vector3d(2.69,20,0));
        movementSync.velocity.set(new Vector3d(.4,0,0));
        movementSync.snapshotInput();
        assertEquals(0, accepted.get());
        assertNull(movementSync.pollBeforeMovementPacket());
    }
    @Test void beyondMaximumDecisionAgeIsRejected() throws Exception {
        world();
        var accepted = new AtomicInteger();
        var envelope = envelope(new Object(), accepted);
        for (int i = 0; i < 9; i++) physicalTick();
        movementSync.submitFlightControls(envelope);
        movementSync.snapshotInput();
        assertEquals(0, accepted.get());
    }
    @Test void delayedTurnAvoidsRealMidrouteWallAndLeaseEndsInSupportedTouchdown() throws Exception {
        var stone = new xin.bbtt.Block.BlockState("minecraft:stone", 1, java.util.Map.of(),
                "block", 1.5, true, "STONE");
        for (int delay = 3; delay <= 8; delay++) {
            world();
            var field = xin.bbtt.MovementSync.class.getDeclaredField("world");
            field.setAccessible(true);
            field.set(movementSync, new World() {
                @Override public boolean chunkLoaded(int x, int z) { return true; }
                @Override public xin.bbtt.Block.BlockState getBlockStateAt(Vector3d p) {
                    return p.y < 0 || p.z >= 8 && p.z < 9 && p.x >= -2 && p.x < 2 && p.y < 30
                            ? stone : super.getBlockStateAt(p);
                }
            });
            movementSync.yaw.set(0f);
            movementSync.pitch.set(0f);
            var accepted = new AtomicInteger();
            var turn = new ElytraFlightController.Decision(-90, 0, false,
                    ElytraFlightController.Phase.CRUISE);
            var envelope = FlightControlAdmission.prepare(movementSync, new Object(),
                    FlightControlAdmission.snapshot(movementSync), turn, null, accepted::incrementAndGet);
            assertTrue(envelope.steps().size() > 6, "requires finite prepared continuation");
            for (int tick = 0; tick < delay; tick++) physicalTick();
            movementSync.submitFlightControls(envelope);
            boolean landed = false;
            for (int tick = 0; tick < 600; tick++) {
                var input = movementSync.snapshotInput();
                var state = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(),
                        false, movementSync.fallDistance.get(), PlayerPose.GLIDING);
                var result = new GlidingPhysics(movementSync.yaw.get(), movementSync.pitch.get(), false,
                        input, movementSync.getMovementParameters()).step(movementSync.getWorld(), state,
                        PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
                assertFalse(result.collidedX(), "delay=" + delay + " tick=" + tick);
                assertFalse(result.collidedZ(), "delay=" + delay + " tick=" + tick);
                movementSync.position.set(result.position());
                movementSync.velocity.set(result.velocity());
                movementSync.fallDistance.set(result.fallDistance());
                movementSync.onGround.set(result.onGround());
                if (result.onGround()) { landed = true; break; }
            }
            assertEquals(1, accepted.get(), "delayed turn accepted at delay=" + delay);
            assertTrue(landed, "no worker refresh: continuation must terminate at delay=" + delay);
            assertTrue(movementSync.position.get().x > 2, "actually steered around the wall");
            movementSync.revokeFlightControls(envelope.owner());
        }
    }

}
