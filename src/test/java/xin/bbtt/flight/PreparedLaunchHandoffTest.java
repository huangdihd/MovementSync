package xin.bbtt.flight;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponents;
import org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponentTypes;
import org.geysermc.mcprotocollib.protocol.data.game.item.component.Fireworks;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.FireworkSchedule;
import xin.bbtt.physics.FlightControlAdmission;
import xin.bbtt.physics.HazardRecovery;
import xin.bbtt.physics.LowAltitudePlanner;
import xin.bbtt.physics.MovementParameters;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerPose;
import xin.bbtt.physics.WaypointFlightTarget;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

class PreparedLaunchHandoffTest extends ElytraCommandTestSupport {
    private static final BlockState STONE = new BlockState("minecraft:stone", 1, Map.of(),
            "block", 1.5, true, "STONE");
    private static final HazardRecovery.Context RESOURCES =
            new HazardRecovery.Context(new int[] {1}, 0, false, null);
    private final Object owner = new Object();
    private final AtomicInteger accepted = new AtomicInteger();

    private static final class MutableWorld extends World {
        private boolean floor;
        private boolean forwardWall;
        private boolean sideWall;

        @Override public boolean chunkLoaded(int x, int z) { return true; }

        @Override public BlockState getBlockStateAt(Vector3d position) {
            if (floor && position.y < 64
                    || forwardWall && position.z >= 2 && position.z < 3
                    || sideWall && position.x >= 2 && position.x < 3) return STONE;
            return super.getBlockStateAt(position);
        }
    }

    private record Fixture(MutableWorld world, FlightControlAdmission.Source source,
            WaypointFlightTarget route, Vector3d goal, LocalGotoController.PreparedLaunch launch) {}

    private Fixture prepare(boolean lowLaunch) throws Exception {
        var world = new MutableWorld();
        world.floor = lowLaunch;
        var initial = lowLaunch
                ? new PhysicsState(new Vector3d(16.53, 65.08, 8.5),
                        new Vector3d(.018, -.092, 0), false, 1, PlayerPose.GLIDING)
                : new PhysicsState(new Vector3d(.5, 100, .5),
                        new Vector3d(0, 0, .3), false, 0, PlayerPose.GLIDING);
        var goal = lowLaunch ? new Vector3d(60, 64, 8.5) : new Vector3d(.5, 100, 80);
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
        movementSync.position.set(initial.position());
        movementSync.velocity.set(initial.velocity());
        movementSync.fallDistance.set(initial.fallDistance());
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        inventory[36] = item("firework_rocket", 1);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        var source = FlightControlAdmission.snapshot(movementSync);
        var route = LowAltitudePlanner.planPursuit(world, source.state(), goal, MovementParameters.DEFAULT);
        assertNotNull(route, "fixture must have a planner-backed restrictive corridor");
        var launch = LocalGotoController.prepareLaunch(world, MovementParameters.DEFAULT,
                source.state(), route, goal, lowLaunch ? -90 : 0, 0, RESOURCES);
        assertNotNull(launch, "fixture must prove a complete launch before testing handoff gates");
        return new Fixture(world, source, route, goal, launch);
    }

    private boolean publish(LocalGotoController controller, Fixture fixture) {
        return controller.publishLaunch(movementSync, owner, fixture.source(), fixture.launch(),
                fixture.route(), fixture.goal(), RESOURCES, accepted::incrementAndGet);
    }

    @Test void lowLaunchPublishesAndOrdinaryAdmissionExecutesExactlyOnceWithoutWorkerRequest() throws Exception {
        var fixture = prepare(true);
        try (var controller = new LocalGotoController()) {
            assertTrue(publish(controller, fixture));
            assertEquals(0, accepted.get(), "preparation and publication cannot execute the action");
            assertNull(controller.latest(), "prepared handoff must not require an asynchronous request");
            var envelope = pendingEnvelope();
            var lifetime = FireworkSchedule.lifetimeBounds(1);
            assertEquals(lifetime, envelope.rocketLifetime());
            assertTrue(envelope.requireAirborne());
            assertTrue(envelope.steps().get(0).control().firework());
            assertTrue(envelope.steps().size() + envelope.recovery().size() + envelope.handoff().size()
                    > lifetime.maximumPoweredTicks(), "proof must outlive every legal rocket lifetime");
            assertTrue(envelope.corridor().sameCorridor(fixture.route()));
            assertTrue(envelope.corridor().terrainValidated());

            movementSync.snapshotInput();

            assertEquals(1, accepted.get());
            assertTrue(movementSync.flightControlOwnership().active());
            movementSync.snapshotInput();
            flightControls().submit(envelope);
            movementSync.snapshotInput();
            assertEquals(1, accepted.get(), "reconsuming or resubmitting the envelope cannot repeat its action");
            assertNull(controller.latest());
        }
    }

    @Test void removedHotbarRocketRejectsAdmissionAfterSuccessfulPublication() throws Exception {
        var fixture = prepare(false);
        try (var controller = new LocalGotoController()) {
            assertTrue(publish(controller, fixture));
            movementSync.inventoryManager.setSlot(0, 36, null);
            assertEquals(0, movementSync.countHotbarFireworks());

            movementSync.snapshotInput();

            assertEquals(0, accepted.get(), "removed rocket must prevent the launch action");
        }
    }

    @Test void replacedRocketLifetimeRejectsAdmissionAfterSuccessfulPublication() throws Exception {
        var fixture = prepare(false);
        try (var controller = new LocalGotoController()) {
            assertTrue(publish(controller, fixture));
            var components = new DataComponents(new HashMap<>());
            components.put(DataComponentTypes.FIREWORKS, new Fireworks(2, List.of()));
            movementSync.inventoryManager.setSlot(0, 36,
                    new ItemStack(inventory[36].getId(), 1, components));
            assertArrayEquals(new int[] {2}, movementSync.inventoryManager.getHotbarFireworkFlightDurations());

            movementSync.snapshotInput();

            assertEquals(0, accepted.get(), "replacement rocket must not reuse a different lifetime proof");
        }
    }

    @Test void acquiredFireworkRequestLatchRejectsAdmissionAfterSuccessfulPublication() throws Exception {
        var fixture = prepare(false);
        try (var controller = new LocalGotoController()) {
            assertTrue(publish(controller, fixture));
            assertTrue(movementSync.useFirework(), "real inventory-backed request must acquire the shared latch");
            assertFalse(movementSync.fireworkRequestReady());
            assertEquals(1, movementSync.countHotbarFireworks(), "queued request must not empty the inventory");

            movementSync.snapshotInput();

            assertEquals(0, accepted.get(), "another pending request must prevent the launch action");
        }
    }

    @Test void emptyHotbarRefusesPublicationDespiteValidResources() throws Exception {
        var fixture = prepare(false);
        movementSync.inventoryManager.setSlot(0, 36, null);
        assertEquals(0, movementSync.countHotbarFireworks());
        try (var controller = new LocalGotoController()) {
            assertFalse(publish(controller, fixture), "resource snapshot cannot replace actual rocket inventory");
            assertNoPublication();
        }
    }

    @Test void acquiredFireworkRequestLatchRefusesPublicationDespiteValidResources() throws Exception {
        var fixture = prepare(false);
        assertTrue(movementSync.useFirework(), "real inventory-backed request must acquire the shared latch");
        assertFalse(movementSync.fireworkRequestReady());
        assertEquals(1, movementSync.countHotbarFireworks(), "queued request must not empty the inventory");
        try (var controller = new LocalGotoController()) {
            assertFalse(publish(controller, fixture), "resource snapshot cannot bypass the shared request latch");
            assertNoPublication();
        }
    }

    @Test void changedRocketLifetimeRefusesPreparedLaunch() throws Exception {
        var fixture = prepare(false);
        try (var controller = new LocalGotoController()) {
            assertFalse(controller.publishLaunch(movementSync, owner, fixture.source(), fixture.launch(),
                    fixture.route(), fixture.goal(),
                    new HazardRecovery.Context(new int[] {2}, 0, false, null), accepted::incrementAndGet));
            assertNoPublication();
        }
    }

    @Test void unavailableOrAttachedRocketCannotReusePreparedLaunch() throws Exception {
        var fixture = prepare(false);
        var unavailable = List.of(
                new HazardRecovery.Context(new int[0], 0, false, null),
                new HazardRecovery.Context(new int[] {1}, 1, false, null),
                new HazardRecovery.Context(new int[] {1}, 0, true, null));
        try (var controller = new LocalGotoController()) {
            for (var resources : unavailable) {
                assertFalse(controller.publishLaunch(movementSync, owner, fixture.source(), fixture.launch(),
                        fixture.route(), fixture.goal(), resources, accepted::incrementAndGet));
                assertNoPublication();
            }
        }
    }

    @Test void expiredSourceCannotPublishPreparedLaunch() throws Exception {
        var fixture = prepare(false);
        for (int tick = 0; tick < 9; tick++) movementSync.completePhysicsStep();
        try (var controller = new LocalGotoController()) {
            assertFalse(publish(controller, fixture));
            assertNoPublication();
        }
    }

    @Test void wrongOwnerCannotPublishEvenWithCurrentSource() throws Exception {
        var fixture = prepare(false);
        try (var controller = new LocalGotoController()) {
            assertFalse(controller.publishLaunch(movementSync, new Object(), fixture.source(), fixture.launch(),
                    fixture.route(), fixture.goal(), RESOURCES, accepted::incrementAndGet));
            assertNoPublication();
        }
    }

    @Test void invalidatedEpochCannotPublishPreparedLaunch() throws Exception {
        var fixture = prepare(false);
        movementSync.invalidateFlightControls();
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        assertNotEquals(fixture.source().epoch(), movementSync.flightControlEpoch());
        try (var controller = new LocalGotoController()) {
            assertFalse(publish(controller, fixture));
            assertNoPublication();
        }
    }

    @Test void obstructionAddedAfterPreparationRejectsPublication() throws Exception {
        var fixture = prepare(false);
        fixture.world().forwardWall = true;
        try (var controller = new LocalGotoController()) {
            assertFalse(publish(controller, fixture), "prepared controls are not a cached terrain certificate");
            assertNoPublication();
        }
    }

    @Test void incompatibleRouteCannotBorrowPreparedControls() throws Exception {
        var fixture = prepare(false);
        var start = fixture.source().state().position();
        var sidewaysGoal = new Vector3d(start).add(40, 0, 0);
        var sideways = new WaypointFlightTarget(List.of(start, sidewaysGoal), sidewaysGoal, false, false, .1);
        try (var controller = new LocalGotoController()) {
            assertFalse(controller.publishLaunch(movementSync, owner, fixture.source(), fixture.launch(),
                    sideways, sidewaysGoal, RESOURCES, accepted::incrementAndGet));
            assertNoPublication();
        }
    }

    @Test void shiftedLiveCollisionRejectsOrdinaryAdmissionDespiteSuccessfulPublication() throws Exception {
        var fixture = prepare(false);
        var shifted = fixture.source().state().position().add(2, 0, 0);
        assertTrue(fixture.route().contains(shifted, shifted), "shift alone must remain inside the planned space");
        try (var controller = new LocalGotoController()) {
            assertTrue(publish(controller, fixture));
            var envelope = pendingEnvelope();
            assertTrue(envelope.corridor().sameCorridor(fixture.route()));
            assertTrue(envelope.corridor().terrainValidated());
            fixture.world().sideWall = true;
            assertFalse(envelope.corridor().contains(shifted, shifted),
                    "published corridor must retain live swept-volume geometry");
            movementSync.position.set(shifted);
            movementSync.completePhysicsStep();

            movementSync.snapshotInput();

            assertEquals(0, accepted.get(), "live collision must prevent the launch action");
            assertFalse(movementSync.flightControlOwnership().active());
            assertNotNull(movementSync.getFlightControlFailure());
            assertNull(controller.latest());
        }
    }

    private void assertNoPublication() throws Exception {
        assertEquals(0, accepted.get());
        assertNull(pendingEnvelope());
        assertFalse(movementSync.flightControlOwnership().active());
    }

    private FlightControlAdmission.Envelope pendingEnvelope() throws Exception {
        Field pending = FlightControlAdmission.class.getDeclaredField("pending");
        pending.setAccessible(true);
        return (FlightControlAdmission.Envelope) pending.get(flightControls());
    }

    private FlightControlAdmission flightControls() throws Exception {
        Field controls = MovementSync.class.getDeclaredField("flightControls");
        controls.setAccessible(true);
        return (FlightControlAdmission) controls.get(movementSync);
    }
}
