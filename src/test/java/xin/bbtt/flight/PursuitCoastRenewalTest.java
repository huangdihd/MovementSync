package xin.bbtt.flight;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import org.cloudburstmc.nbt.NbtMap;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.listeners.RegistryDataListener;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

/** A short coast must not hide a viable maneuvering rocket episode under a low roof. */
final class PursuitCoastRenewalTest extends ElytraCommandTestSupport {
    private static final BlockState STONE = new BlockState(
            "minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState(
            "minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");
    private List<NbtMap> dimensions;
    private List<NbtMap> savedDimensions;
    private Field dimensionIndex;
    private int savedIndex;

    @BeforeEach @SuppressWarnings("unchecked")
    void installDimension() throws Exception {
        Field field = RegistryDataListener.class.getDeclaredField("dimensionTypes");
        field.setAccessible(true);
        dimensions = (List<NbtMap>) field.get(null);
        savedDimensions = new ArrayList<>(dimensions);
        dimensionIndex = RegistryDataListener.class.getDeclaredField("currentDimensionIndex");
        dimensionIndex.setAccessible(true);
        savedIndex = dimensionIndex.getInt(null);
        dimensions.clear();
        dimensions.add(NbtMap.builder().putBoolean("has_ceiling", true)
                .putInt("min_y", 0).putInt("height", 256).putInt("logical_height", 128).build());
        dimensionIndex.setInt(null, 0);
    }

    @AfterEach void restoreDimension() throws Exception {
        dimensions.clear();
        dimensions.addAll(savedDimensions);
        dimensionIndex.setInt(null, savedIndex);
    }

    private static final class UnderroofWorld extends World {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public BlockState getBlockStateAt(Vector3d position) {
            return position.y < 61 || position.y >= 76 ? STONE : AIR;
        }
    }

    @Test void poorStraightPoweredScoreStillRequestsAnEarlyProvenEpisode() throws Exception {
        exercise(true, 0, true, true);
    }

    @Test void unavailableRocketCannotRenewCoast() throws Exception {
        exercise(false, 0, true, false);
    }

    @Test void cooldownCannotRenewCoast() throws Exception {
        exercise(true, 10, true, false);
    }

    @Test void staticRouteDoesNotEnablePursuitRenewal() throws Exception {
        exercise(true, 0, false, false);
    }

    private void exercise(boolean available, int cooldownTicks, boolean pursuing,
            boolean expectedRocket) throws Exception {
        var world = new UnderroofWorld();
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
        // Rounded physical state from the moving-target lifecycle at tick 182, well before
        // its tick-220 touchdown; the world has the same floor at 60 and roof at 76.
        movementSync.position.set(new Vector3d(5164, 64.92, 26.80));
        movementSync.velocity.set(new Vector3d(.3985, -.08513, -.02540));
        movementSync.fallDistance.set(1.08513);
        movementSync.yaw.set(-1128.5884f);
        movementSync.pitch.set(-30f);
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        if (available) inventory[36] = item("firework_rocket", 64);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        Object owner = new Object();
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        var source = FlightControlAdmission.snapshot(movementSync);
        var goal = new Vector3d(5164.4, 61, 28.5);
        var route = new WaypointFlightTarget(List.of(new Vector3d(5163, 65.18, 26.96),
                new Vector3d(5179, 63.13, 28.5)), goal, false);
        var tracking = pursuing ? new LocalLookaheadSolver.MovingTarget(goal, new Vector3d(.8, 0, 0)) : null;
        var resources = new HazardRecovery.Context(available ? new int[] {1} : new int[0],
                cooldownTicks, false, null);
        var accepted = new AtomicInteger();
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route, goal, true, () -> {
                accepted.incrementAndGet();
                assertTrue(movementSync.useFirework(), "admitted episode must queue a real rocket request");
            }, tracking, resources, true));
            assertTrue(controller.awaitIdle(10, TimeUnit.SECONDS));
            var result = controller.latest();
            assertNotNull(result);
            assertEquals(LocalGotoController.Status.SAFE_PREFIX, result.status());
            assertEquals(expectedRocket, result.rocket(), () -> "result=" + result);
            if (expectedRocket) {
                assertEquals(12, result.unpowered().prefix().size(), "short coast remains viable");
                assertNotNull(result.powered());
                assertEquals(12, result.powered().prefix().size(), "both seed horizons must be complete");
                assertTrue(result.powered().score() < result.unpowered().score(),
                        "regression must reject score-only rocket selection");
                assertTrue(source.state().position().y - 61 > 3, "renewal must occur before emergency altitude");
                var envelope = pendingEnvelope();
                int lifetime = FireworkSchedule.lifetimeBounds(1).maximumPoweredTicks() + 2;
                var coast = LocalGotoController.replay(world, movementSync.getMovementParameters(),
                        source.state(), result.unpowered().prefix().get(0).control(), false,
                        lifetime, goal, false);
                assertTrue(coast.steps().size() < lifetime,
                        "ordinary coast cannot preserve airborne clearance for a request lifetime");
                assertNotNull(envelope.rocketLifetime());
                assertTrue(envelope.requireAirborne());
                assertEquals(lifetime, envelope.steps().size() + envelope.recovery().size()
                        + envelope.handoff().size(), "full request lifetime including bridge must be proven");
                assertTrue(envelope.steps().stream().noneMatch(step -> step.expected().onGround()));
                assertTrue(envelope.recovery().stream().noneMatch(step -> step.expected().onGround()));
                assertTrue(envelope.handoff().stream().noneMatch(step -> step.expected().onGround()));
            }
            movementSync.snapshotInput();
            assertEquals(expectedRocket ? 1 : 0, accepted.get(), "live admission must own request dispatch");
            int requests = 0;
            for (var packet = movementSync.pollBeforeMovementPacket(); packet != null;
                    packet = movementSync.pollBeforeMovementPacket()) {
                if (packet instanceof ServerboundUseItemPacket) requests++;
                assertEquals(MovementSync.BeforeMovementDispatch.SENT,
                        movementSync.sendBeforeMovementPacket(packet, packet));
            }
            assertEquals(expectedRocket ? 1 : 0, requests);
            assertEquals(requests, packets.stream().filter(ServerboundUseItemPacket.class::isInstance).count(),
                    "request must reach the recording protocol session");
            assertFalse(movementSync.hasAttachedFirework(), "request alone is not an attachment");
        }
    }

    private FlightControlAdmission.Envelope pendingEnvelope() throws Exception {
        Field controls = MovementSync.class.getDeclaredField("flightControls");
        controls.setAccessible(true);
        Field pending = FlightControlAdmission.class.getDeclaredField("pending");
        pending.setAccessible(true);
        return assertInstanceOf(FlightControlAdmission.Envelope.class, pending.get(controls.get(movementSync)));
    }
}
