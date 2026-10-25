package xin.bbtt.flight;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Synthetic loaded support; not a reconstruction of the field's unrecorded terrain. */
class P5fRealLandingAdmissionTest extends ElytraCommandTestSupport {
    private World landingWorld() {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                return p.y < 60 ? new BlockState("stone", 1, Map.of(), "block", 1.5, true, "STONE")
                        : super.getBlockStateAt(p);
            }
        };
    }

    @Test void highOverGoalWith64RocketsMustDescendAndActuallyLandWithoutMoreThrust() throws Exception {
        land(110, false);
    }

    @Test void normalArrivalWith64RocketsStillLandsWithoutMoreThrust() throws Exception {
        land(64, false);
    }

    @Test void attachedArrivalCompletesAfterExistingRocketExpiresWithoutStacking() throws Exception {
        land(110, true);
    }

    @Test void liveDeviationCannotRebaseAnAlreadyPublishedTerminalExecutionCorridor() throws Exception {
        World world = landingWorld();
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, world);
        var goal = new Vector3d(175, 60, -32);
        var initial = new Vector3d(175, 64, -36);
        movementSync.position.set(initial);
        movementSync.velocity.set(new Vector3d(0, -.1, .4));
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        Object owner = new Object();
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        var source = FlightControlAdmission.snapshot(movementSync);
        var route = new WaypointFlightTarget(List.of(initial, goal), goal, false);
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route, goal));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertEquals(LocalGotoController.Status.SAFE_PREFIX, controller.latest().status());
            movementSync.position.set(new Vector3d(initial).add(.25, 0, 0));

            movementSync.snapshotInput();

            assertNotNull(movementSync.getFlightControlFailure(),
                    "a live displacement must not be absorbed into the published terminal replay");
        }
    }

    private void land(double height, boolean initiallyAttached) throws Exception {
        World world = landingWorld();
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, world);
        inventory[36] = item("firework_rocket", 64);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        var goal = new Vector3d(175, 60, -32);
        var initial = new Vector3d(175, height, -36);
        movementSync.position.set(initial);
        movementSync.velocity.set(new Vector3d(0, -.1, .4));
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        if (initiallyAttached) {
            var rocket = new xin.bbtt.Entity.Entity(43, new java.util.UUID(0, 43),
                    org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType.FIREWORK_ROCKET,
                    initial.x, initial.y, initial.z, 0, 0, 0, new Vector3d());
            rocket.getMetadata().put("attached_to_target", java.util.OptionalInt.of(42));
            world.getEntities().put(43, rocket);
        }
        Object owner = new Object();
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        var route = new WaypointFlightTarget(List.of(initial, goal), goal, false);
        int[] rockets = new int[64];
        Arrays.fill(rockets, 1);
        try (var controller = new LocalGotoController()) {
            for (int tick = 0; tick < 600; tick++) {
                if (tick == 20) world.getEntities().remove(43);
                boolean attached = movementSync.hasAttachedFirework();
                assertEquals(initiallyAttached && tick < 20, attached);
                var source = FlightControlAdmission.snapshot(movementSync);
                if (LocalGotoController.normalLanding(world, source.state(), goal)) {
                    assertTrue(tick > 0, "must execute admissions before completion");
                    assertEquals(64, movementSync.inventoryManager.getInventory()[36].getAmount());
                    return;
                }
                assertTrue(controller.request(movementSync, owner, source, route, goal, true,
                        () -> fail("arrival must not spend a rocket"), null,
                        new HazardRecovery.Context(rockets, 0, attached, movementSync.getLandingMemory()), true, true));
                controller.awaitIdle(5, TimeUnit.SECONDS);
                assertFalse(controller.latest().rocket(), "arrival must not request another rocket at " + source.state());
                assertEquals(LocalGotoController.Status.SAFE_PREFIX, controller.latest().status());
                // Consume the published envelope at the production physical-input boundary.
                // A sentinel proves the consumer, not direct pending-envelope reflection,
                // supplied the attitude used for this physics step.
                movementSync.pitch.set(89f);
                var input = movementSync.snapshotInput();
                assertNotEquals(89f, movementSync.pitch.get(), "admission must apply landing control at tick " + tick
                        + ": state=" + source.state() + " result=" + controller.latest()
                        + " failure=" + movementSync.getFlightControlFailure());
                assertNull(movementSync.getFlightControlFailure(), "landing admission must remain valid");
                var next = new GlidingPhysics(movementSync.yaw.get(), movementSync.pitch.get(), attached, input,
                        MovementParameters.DEFAULT).step(world, source.state(), PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
                for (var packet = movementSync.pollBeforeMovementPacket(); packet != null;
                        packet = movementSync.pollBeforeMovementPacket()) {
                    assertFalse(packet instanceof org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket,
                            "admission/fallback must not queue a rocket request");
                }
                assertTrue(packets.stream().noneMatch(org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket.class::isInstance),
                        "physical boundary must not request an extra rocket");
                assertFalse(next.collidedX() || next.collidedZ());
                movementSync.position.set(next.position());
                movementSync.velocity.set(next.velocity());
                movementSync.onGround.set(next.onGround());
                movementSync.fallDistance.set(next.fallDistance());
                movementSync.completePhysicsStep();

            }
            fail("must physically reach supported normal arrival, not orbit forever: " + movementSync.position.get());
        }
    }
}
