package xin.bbtt;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.OptionalInt;
import java.util.UUID;
import org.geysermc.mcprotocollib.network.ClientSession;
import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.EntityMetadata;
import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.MetadataTypes;
import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.type.ObjectEntityMetadata;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundSetEntityDataPacket;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PlayerState;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.ServerboundClientTickEndPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundPlayerCommandPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundSetCarriedItemPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket;
import org.joml.Vector3d;
import xin.bbtt.Entity.Entity;
import xin.bbtt.Entity.EntityRegistry;
import xin.bbtt.listeners.EntityPacketListener;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.events.ReceivePacketEvent;
import xin.bbtt.physics.FlightControlAdmission;
import xin.bbtt.physics.PlayerInput;
import xin.bbtt.tasks.updateMotionTask;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

final class FireworkRequestLatchTest extends ElytraCommandTestSupport {
    @Test void sessionResetDiscardsUnsentRequestAndSlotSelection() {
        inventory[36] = null;
        inventory[40] = item("firework_rocket", 3);
        movementSync.setGliding(true);
        assertEquals(0, movementSync.inventoryManager.getHeldSlot());
        assertTrue(movementSync.useFirework());
        assertFalse(movementSync.fireworkRequestReady());

        movementSync.suspendNavigationForRecovery();

        assertNull(movementSync.pollBeforeMovementPacket());
        assertTrue(movementSync.fireworkRequestReady());
        assertEquals(0, movementSync.inventoryManager.getHeldSlot());
    }

    @Test void throwingSendRetainsRequestUntilActualDispatch() throws Exception {
        prepareRocket();
        assertTrue(movementSync.useFirework());
        ServerboundUseItemPacket request = assertInstanceOf(
                ServerboundUseItemPacket.class, movementSync.pollBeforeMovementPacket());
        Field session = Bot.class.getDeclaredField("session");
        session.setAccessible(true);
        Object connected = session.get(Bot.INSTANCE);
        Object throwing = Proxy.newProxyInstance(ClientSession.class.getClassLoader(),
                new Class<?>[]{ClientSession.class}, (proxy, method, arguments) -> {
                    if (method.getName().equals("isConnected")) return true;
                    if (method.getName().equals("send")) throw new IllegalStateException("synthetic send failure");
                    if (method.getReturnType() == boolean.class) return false;
                    return null;
                });
        session.set(Bot.INSTANCE, throwing);
        try {
            assertEquals(MovementSync.BeforeMovementDispatch.RETRY,
                    movementSync.sendBeforeMovementPacket(request, request));
            for (int tick = 0; tick < 40; tick++) movementSync.completePhysicsStep();
            assertFalse(movementSync.fireworkRequestReady(), "failed send must retain the pending latch");
        } finally {
            session.set(Bot.INSTANCE, connected);
        }

        long dispatchTick = movementSync.physicsTickVersion();
        assertEquals(MovementSync.BeforeMovementDispatch.SENT,
                movementSync.sendBeforeMovementPacket(request, request));
        for (int tick = 1; tick < 40; tick++) {
            movementSync.completePhysicsStep();
            assertFalse(movementSync.fireworkRequestReady(), "successful send cooldown tick " + tick);
        }
        movementSync.completePhysicsStep();
        assertEquals(dispatchTick + 40, movementSync.physicsTickVersion());
        assertTrue(movementSync.fireworkRequestReady());
    }

    @Test void abandonedUnsentRequestDoesNotConsumeCooldown() {
        prepareRocket();
        assertTrue(movementSync.useFirework());
        movementSync.cancelNavigation();
        assertNull(movementSync.pollBeforeMovementPacket());
        assertTrue(movementSync.fireworkRequestReady());
        assertTrue(movementSync.useFirework());
    }

    @Test void respawnDiscardsUnsentRequestWithoutConsumingCooldown() {
        prepareRocket();
        assertTrue(movementSync.useFirework());
        movementSync.onPlayerRespawn();
        movementSync.setGliding(true);
        assertNull(movementSync.pollBeforeMovementPacket());
        assertTrue(movementSync.useFirework());
    }

    @Test void staleEpochDiscardReleasesUnsentRequest() {
        prepareRocket();
        movementSync.acceptFlightAction(() -> assertTrue(movementSync.useFirework()));
        movementSync.invalidateFlightControls();
        assertNull(movementSync.pollBeforeMovementPacket());
        assertTrue(movementSync.useFirework());
    }

    @Test void dispatchedRequestKeepsCooldownAcrossCancellationAndRespawn() {
        prepareRocket();
        assertTrue(movementSync.useFirework());
        dispatchNextFirework();
        movementSync.cancelNavigation();
        movementSync.onPlayerRespawn();
        movementSync.setGliding(true);
        assertFalse(movementSync.fireworkRequestReady());
        assertFalse(movementSync.useFirework());
        for (int tick = 0; tick < 40; tick++) movementSync.completePhysicsStep();
        assertTrue(movementSync.useFirework());
    }

    @Test void invalidatedAcceptedRequestNeverDispatchesOrCreatesAnAcknowledgement() throws Exception {
        try (var harness = new ActionBoundaryHarness()) {
            inventory[36] = null;
            inventory[40] = item("firework_rocket", 3);
            movementSync.inventoryManager.setContainerItems(0, inventory);
            assertEquals(0, movementSync.inventoryManager.getHeldSlot());
            movementSync.claimFlightControlOwner(new Object(), FlightControlAdmission.Mode.DIRECT);
            long epoch = movementSync.flightControlEpoch();
            movementSync.acceptFlightAction(() -> assertTrue(movementSync.useFirework()));
            assertFalse(movementSync.fireworkRequestReady());
            assertEquals(0, harness.dispatches());
            movementSync.invalidateFlightControls();
            assertTrue(movementSync.flightControlEpoch() > epoch);

            for (int tick = 0; tick < 4; tick++) {
                harness.tick();
                assertEquals(0, harness.dispatches(), "stale action must not reach session.send");
                assertFalse(movementSync.hasAttachedFirework(), "no send means no scheduled acknowledgement");
                assertTrue(movementSync.fireworkRequestReady(), "discard must release the unsent latch");
            }
            assertTrue(packets.stream().noneMatch(ServerboundSetCarriedItemPacket.class::isInstance),
                    "stale firework slot selection must not reach session.send");
            assertEquals(0, movementSync.inventoryManager.getHeldSlot());
            assertEquals(3, inventory[40].getAmount());
        }
    }

    @Test void freshOwnerReplacesDiscardedRequestOnceAndCooldownSurvivesOwnerChanges() throws Exception {
        try (var harness = new ActionBoundaryHarness()) {
            movementSync.claimFlightControlOwner(new Object(), FlightControlAdmission.Mode.DIRECT);
            movementSync.acceptFlightAction(() -> assertTrue(movementSync.useFirework()));
            long staleEpoch = movementSync.flightControlEpoch();
            movementSync.claimFlightControlOwner(new Object(), FlightControlAdmission.Mode.LOCAL);
            assertTrue(movementSync.flightControlEpoch() > staleEpoch);
            harness.tick();
            assertEquals(0, harness.dispatches());
            assertFalse(movementSync.hasAttachedFirework());
            assertTrue(movementSync.fireworkRequestReady());

            long requestTick = movementSync.physicsTickVersion();
            movementSync.acceptFlightAction(() -> assertTrue(movementSync.useFirework()));
            movementSync.acceptFlightAction(() -> assertFalse(movementSync.useFirework(), "same-tick duplicate"));
            assertEquals(requestTick, movementSync.physicsTickVersion());
            harness.tick();
            assertEquals(1, harness.dispatches());
            assertFalse(movementSync.hasAttachedFirework(), "dispatch alone is not attachment");

            for (int elapsed = 1; elapsed < 40; elapsed++) {
                assertEquals(requestTick + elapsed, movementSync.physicsTickVersion());
                movementSync.claimFlightControlOwner(new Object(), FlightControlAdmission.Mode.DIRECT);
                assertFalse(movementSync.fireworkRequestReady(), "cooldown tick " + elapsed);
                movementSync.acceptFlightAction(() -> assertFalse(movementSync.useFirework()));
                assertEquals(elapsed >= 3, movementSync.hasAttachedFirework(),
                        "acknowledgement arrives two completed ticks after dispatch, independent of owner");
                harness.tick();
                assertEquals(1, harness.dispatches(), "owner changes must not duplicate the dispatched request");
            }
            assertEquals(requestTick + 40, movementSync.physicsTickVersion());
            assertTrue(movementSync.fireworkRequestReady());
            // Expire the simulated server entity before testing a second legal request.
            movementSync.getWorld().getEntities().remove(ActionBoundaryHarness.FIREWORK_ID);
            movementSync.claimFlightControlOwner(new Object(), FlightControlAdmission.Mode.LOCAL);
            movementSync.acceptFlightAction(() -> assertTrue(movementSync.useFirework()));
            movementSync.acceptFlightAction(() -> assertFalse(movementSync.useFirework()));
            harness.tick();
            assertEquals(2, harness.dispatches(), "exactly one replacement at the forty-tick boundary");
            assertFalse(movementSync.hasAttachedFirework(), "second dispatch also waits for acknowledgement");
            assertEquals(3, inventory[36].getAmount(), "requests do not mutate server-authoritative inventory");
        }
    }

    @Test void retainedAcceptedEpochDispatchesOnceAndAttachesOnlyOnFixedAcknowledgementTick() throws Exception {
        try (var harness = new ActionBoundaryHarness()) {
            Object owner = new Object();
            movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.DIRECT);
            long epoch = movementSync.flightControlEpoch();
            movementSync.acceptFlightAction(() -> assertTrue(movementSync.useFirework()));
            movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.DIRECT);
            assertEquals(epoch, movementSync.flightControlEpoch(), "same-owner renewal is the no-discard control");
            assertEquals(0, harness.dispatches());
            assertFalse(movementSync.hasAttachedFirework());
            harness.tick();
            assertEquals(1, harness.dispatches());
            assertFalse(movementSync.hasAttachedFirework());
            harness.tick();
            assertEquals(1, harness.dispatches());
            assertFalse(movementSync.hasAttachedFirework(), "acknowledgement is not early");
            harness.tick();
            assertEquals(1, harness.dispatches());
            assertTrue(movementSync.hasAttachedFirework());
            harness.tick();
            assertTrue(movementSync.getLastFlightTick().attached(), "next real physics step observes attachment");
            assertEquals(1, harness.dispatches());
            assertFalse(movementSync.useFirework());
            assertEquals(3, inventory[36].getAmount());
        }
    }

    @Test void heldJumpDeferralStartsCooldownAtActualSessionSend() throws Exception {
        try (var harness = new ActionBoundaryHarness()) {
            Object owner = new Object();
            movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.DIRECT);
            movementSync.setInput(PlayerInput.NONE.withJump(true));
            harness.tick();

            movementSync.queueBeforeMovement(new ServerboundPlayerCommandPacket(
                    movementSync.entityId, PlayerState.START_ELYTRA_FLYING));
            movementSync.acceptFlightAction(() -> assertTrue(movementSync.useFirework()));
            harness.tick();
            assertEquals(0, harness.dispatches(), "held jump must defer the action batch");
            assertFalse(movementSync.fireworkRequestReady(), "deferred request remains pending");

            long dispatchTick = movementSync.physicsTickVersion();
            harness.tick();
            assertEquals(1, harness.dispatches(), "request must dispatch on the deployment tick");
            movementSync.claimFlightControlOwner(new Object(), FlightControlAdmission.Mode.LOCAL);
            while (movementSync.physicsTickVersion() < dispatchTick + 39) harness.tick();
            assertFalse(movementSync.fireworkRequestReady(), "actual send +39 remains in cooldown");
            harness.tick();
            assertEquals(dispatchTick + 40, movementSync.physicsTickVersion());
            assertTrue(movementSync.fireworkRequestReady(), "actual send +40 releases cooldown");
        }
    }

    @Test void heldJumpDeferralRetainsAcceptedEpochUntilSend() throws Exception {
        try (var harness = new ActionBoundaryHarness()) {
            movementSync.claimFlightControlOwner(new Object(), FlightControlAdmission.Mode.DIRECT);
            movementSync.setInput(PlayerInput.NONE.withJump(true));
            harness.tick();

            movementSync.queueBeforeMovement(new ServerboundPlayerCommandPacket(
                    movementSync.entityId, PlayerState.START_ELYTRA_FLYING));
            movementSync.acceptFlightAction(() -> assertTrue(movementSync.useFirework()));
            harness.tick();
            assertEquals(0, harness.dispatches());

            movementSync.claimFlightControlOwner(new Object(), FlightControlAdmission.Mode.LOCAL);
            harness.tick();
            assertEquals(0, harness.dispatches(), "deferred stale request must not reach session.send");
            assertTrue(movementSync.fireworkRequestReady(), "discard must release the unsent latch");
        }
    }

    /** Queue/action-boundary fixture only: untouched empty World, not a terrain or n180 reproduction. */
    private final class ActionBoundaryHarness implements AutoCloseable {
        private static final int FIREWORK_ID = 9001;
        private final updateMotionTask task = new updateMotionTask();
        private final Field running = Bot.class.getDeclaredField("running");
        private final Object previousRunning;
        private long acknowledgementTick = Long.MAX_VALUE;

        private ActionBoundaryHarness() throws Exception {
            running.setAccessible(true);
            previousRunning = running.get(Bot.INSTANCE);
            movementSync.getMovementController().pause();
            movementSync.onRecoveryTeleport();
            prepareRocket();
            movementSync.position.set(new Vector3d(.5, 100, .5));
            movementSync.velocity.set(new Vector3d());
            movementSync.onGround.set(false);
            running.set(Bot.INSTANCE, true);
        }

        private long dispatches() {
            return packets.stream().filter(ServerboundUseItemPacket.class::isInstance).count();
        }

        private void tick() {
            long before = movementSync.physicsTickVersion();
            long sentBefore = dispatches();
            long endsBefore = packets.stream().filter(ServerboundClientTickEndPacket.class::isInstance).count();
            task.run();
            assertEquals(before + 1, movementSync.physicsTickVersion(), "must commit a real physics step");
            assertEquals(endsBefore + 1,
                    packets.stream().filter(ServerboundClientTickEndPacket.class::isInstance).count());
            long sent = dispatches() - sentBefore;
            assertTrue(sent == 0 || sent == 1, "at most one actual firework send per tick");
            if (sent == 1) {
                assertEquals(Long.MAX_VALUE, acknowledgementTick, "no overlapping pending acknowledgements");
                acknowledgementTick = movementSync.physicsTickVersion() + 2;
            }
            if (movementSync.physicsTickVersion() == acknowledgementTick) {
                assertTrue(dispatches() > 0, "only a recorded session.send schedules attachment");
                assertFalse(movementSync.hasAttachedFirework());
                Entity rocket = new Entity(FIREWORK_ID, new UUID(0, FIREWORK_ID), EntityType.FIREWORK_ROCKET,
                        .5, 100, .5, 0, 0, 0, new Vector3d());
                movementSync.getWorld().getEntities().put(FIREWORK_ID, rocket);
                int attachmentId = EntityRegistry.Instance.getEntity("FIREWORK_ROCKET")
                        .getMetadataKeys().indexOf("attached_to_target");
                assertTrue(attachmentId >= 0);
                var packet = new ClientboundSetEntityDataPacket(FIREWORK_ID, new EntityMetadata<?, ?>[]{
                        new ObjectEntityMetadata<>(attachmentId, MetadataTypes.OPTIONAL_UNSIGNED_INT,
                                OptionalInt.of(movementSync.entityId))});
                new EntityPacketListener().OnSetEntityMetadata(
                        new ReceivePacketEvent<>(packet, ClientboundSetEntityDataPacket.class));
                assertTrue(movementSync.hasAttachedFirework());
                acknowledgementTick = Long.MAX_VALUE;
            }
        }

        @Override public void close() throws Exception {
            running.set(Bot.INSTANCE, previousRunning);
            movementSync.getWorld().getEntities().remove(FIREWORK_ID);
        }
    }

    private void prepareRocket() {
        inventory[36] = item("firework_rocket", 3);
        movementSync.setGliding(true);
    }

    private ServerboundUseItemPacket dispatchNextFirework() {
        ServerboundUseItemPacket packet = assertInstanceOf(
                ServerboundUseItemPacket.class, movementSync.pollBeforeMovementPacket());
        assertEquals(MovementSync.BeforeMovementDispatch.SENT,
                movementSync.sendBeforeMovementPacket(packet, packet));
        return packet;
    }

    @Test void cooldownStartsWhenDelayedRequestActuallyDispatches() {
        prepareRocket();
        assertTrue(movementSync.useFirework());
        for (int tick = 0; tick < 40; tick++) movementSync.completePhysicsStep();
        assertFalse(movementSync.fireworkRequestReady(), "queued request remains pending");
        dispatchNextFirework();
        assertFalse(movementSync.fireworkRequestReady(), "dispatch must begin a fresh physical cooldown");
        for (int tick = 1; tick < 40; tick++) {
            movementSync.completePhysicsStep();
            assertFalse(movementSync.fireworkRequestReady(), "post-dispatch cooldown tick " + tick);
        }
        movementSync.completePhysicsStep();
        assertTrue(movementSync.fireworkRequestReady());
    }

    @Test void sharedRequestLatchRejectsDuplicatesUntilFortyPhysicalTicks() {
        inventory[36] = item("firework_rocket", 3);
        movementSync.setGliding(true);
        assertTrue(movementSync.fireworkRequestReady());
        assertTrue(movementSync.useFirework());
        dispatchNextFirework();
        assertFalse(movementSync.fireworkRequestReady());
        assertFalse(movementSync.useFirework(), "same-tick requests must share the latch");
        assertNull(movementSync.pollBeforeMovementPacket());
        for (int tick = 0; tick < 39; tick++) movementSync.completePhysicsStep();
        assertFalse(movementSync.fireworkRequestReady());
        assertFalse(movementSync.useFirework(), "tick 39 is still cooling down");
        assertNull(movementSync.pollBeforeMovementPacket());
        movementSync.completePhysicsStep();
        assertTrue(movementSync.fireworkRequestReady());
        assertTrue(movementSync.useFirework());
        dispatchNextFirework();
        assertNull(movementSync.pollBeforeMovementPacket());
        assertFalse(movementSync.hasAttachedFirework(), "a request is not server attachment");
        assertEquals(3, inventory[36].getAmount(), "inventory remains server-authoritative");
    }
}
