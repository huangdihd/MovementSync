package xin.bbtt.movements;

import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PlayerState;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.Hand;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundPlayerCommandPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

/** Synthetic shore: reported height/rotation, not a reconstruction of missing terrain/velocity. */
class AirborneShoreCancellationTest extends ElytraCommandTestSupport {
    private World shore() throws Exception {
        var stone = new BlockState("stone", 1, Map.of(), "block", 1.5, true, "STONE");
        var air = new BlockState("air", 0, Map.of(), "empty", 0, false, "AIR");
        var lava = new BlockState("lava", 0, Map.of("level", "0"), "empty", 0, false, "AIR");
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.y < 28 || p.x < 247 && p.y < 33) return stone;
                return p.x >= 247 && p.y < 29 ? lava : air;
            }
            @Override public boolean isBoxColliding(double x, double y, double z, double X, double Y, double Z) {
                return y < 28 || x < 247 && y < 33;
            }
        };
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, world);
        movementSync.movementController.pause();
        return world;
    }

    private void airborne() {
        movementSync.position.set(new Vector3d(245.5, 34.25, 20.5));
        movementSync.velocity.set(new Vector3d(.34, -.07544, 0));
        movementSync.onGround.set(false);
        movementSync.fallDistance.set(0.0);
        movementSync.yaw.set(-95.4657f);
        movementSync.pitch.set(0f);
    }

    private PhysicsState state() {
        return new PhysicsState(movementSync.position.get(), movementSync.velocity.get(),
                movementSync.onGround.get(), movementSync.fallDistance.get());
    }

    @Test void cancelAndFailureRejectedGlideKeepOwnerThroughRealTouchdownAndStop() throws Exception {
        World world = shore();
        for (boolean cancel : new boolean[] {true, false}) {
            airborne();
            movementSync.applySharedFlags((byte) 0);
            assertTrue(FlightHazards.safeLanding(world, movementSync.position.get()), "no initial body contact");
            assertTrue(GroundHazardExit.airborneRisk(world, state(), movementSync.getMovementParameters()));
            movementSync.queueBeforeMovement(new ServerboundPlayerCommandPacket(42, PlayerState.START_ELYTRA_FLYING));
            movementSync.queueBeforeMovement(new ServerboundUseItemPacket(Hand.MAIN_HAND, 1, 0, 0));
            if (cancel) movementSync.cancelNavigation();
            else movementSync.abandonNavigationRequest(movementSync.getNavigationGeneration(), "rejected deployment");
            assertNull(movementSync.pollBeforeMovementPacket(), "stale flight actions must not restart rejected flight");
            var owner = assertInstanceOf(HazardExitMovement.class, movementSync.movementController.getMovements().get(0));
            boolean groundedBrake = false;
            for (int tick = 0; tick < 121 && !owner.isFinished(); tick++) {
                owner.onTick();
                var input = movementSync.snapshotInput();
                groundedBrake |= movementSync.onGround.get() && input.sneak();
                PhysicsState before = state();
                var next = new WalkingPhysics(input, movementSync.yaw.get(), true, movementSync.getMovementParameters())
                        .step(world, before, PlayerPose.HALF_WIDTH, (double) 1.8f);
                assertTrue(FlightHazards.safeLanding(world, next.position()), "every executed tick avoids hot contact");
                movementSync.position.set(next.position());
                movementSync.velocity.set(next.velocity());
                movementSync.onGround.set(next.onGround());
                movementSync.fallDistance.set(next.fallDistance());
            }
            assertTrue(groundedBrake, "nongliding touchdown must use real crouch braking");
            assertTrue(owner.isFinished());
            assertTrue(GroundedFlightStop.stopped(movementSync));
            assertEquals(PlayerInput.NONE, movementSync.snapshotInput());
        }
    }

    @Test void acceptedGlideUsesFlightSafetyOwnerAndPreservesPendingStartButNotRocket() throws Exception {
        shore();
        for (boolean cancel : new boolean[] {true, false}) {
            airborne();
            movementSync.applySharedFlags((byte) 0x80);
            movementSync.queueBeforeMovement(new ServerboundPlayerCommandPacket(42, PlayerState.START_ELYTRA_FLYING));
            movementSync.queueBeforeMovement(new ServerboundUseItemPacket(Hand.MAIN_HAND, 1, 0, 0));
            if (cancel) movementSync.cancelNavigation();
            else movementSync.abandonNavigationRequest(movementSync.getNavigationGeneration(), "failed flight");
            assertInstanceOf(ElytraFlyMovement.class, movementSync.movementController.getMovements().get(0));
            assertInstanceOf(ServerboundPlayerCommandPacket.class, movementSync.pollBeforeMovementPacket());
            assertNull(movementSync.pollBeforeMovementPacket());
            assertTrue(movementSync.isGliding(), "cancellation cannot fabricate a server glide rejection");
        }
    }
}
