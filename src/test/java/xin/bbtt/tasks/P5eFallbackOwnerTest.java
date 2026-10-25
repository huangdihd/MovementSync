package xin.bbtt.tasks;

import static org.junit.jupiter.api.Assertions.*;
import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.*;
import java.util.concurrent.*;
import org.joml.Vector3d;
import org.junit.jupiter.api.*;
import org.geysermc.mcprotocollib.network.ClientSession;
import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket;
import xin.bbtt.MovementSync;
import xin.bbtt.Entity.Entity;
import xin.bbtt.inventory.ItemRegistry;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.Server;
import xin.bbtt.world.World;
import xin.bbtt.physics.*;
import xin.bbtt.movement.Movement;

class P5eFallbackOwnerTest {
    MovementSync sync;
    updateMotionTask motion;
    List<Object> packets = new ArrayList<>();
    boolean supported, ceiling, lava, unknown;

    @BeforeEach void setup() throws Exception {
        sync = new MovementSync();
        sync.onRecoveryTeleport();
        sync.entityId = 7;
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return !unknown; }
            @Override public xin.bbtt.Block.BlockState getBlockStateAt(Vector3d p) {
                if (lava && p.y < 7) return new xin.bbtt.Block.BlockState("minecraft:lava", 0,
                        Map.of(), "empty", 0, false, "LAVA");
                return super.getBlockStateAt(p);
            }
            @Override public int getBlockAt(Vector3d p) {
                return supported && p.y < 1 || ceiling && p.y >= 12 ? 1 : 0;
            }
        };
        Field field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true); field.set(sync, world);
        bot("running", true);
        Bot.INSTANCE.setServer(Server.Game);
        bot("session", Proxy.newProxyInstance(ClientSession.class.getClassLoader(), new Class<?>[]{ClientSession.class},
                (p, m, a) -> {
                    if (m.getName().equals("isConnected")) return true;
                    if (m.getName().equals("send")) packets.add(a[0]);
                    if (m.getReturnType() == boolean.class) return false;
                    if (m.getReturnType() == int.class) return 0;
                    return null;
                }));
        sync.position.set(new Vector3d(.5, 10, .5));
        sync.velocity.set(new Vector3d(0, -.08, 0));
        sync.onGround.set(false); sync.setGliding(true);
        sync.yaw.set(0f); sync.pitch.set(0f);
        sync.inventoryManager.setContainerItems(0, new ItemStack[46]);
        sync.inventoryManager.getInventory()[36] = new ItemStack(ItemRegistry.Instance.getItem("firework_rocket").getId(), 8);
        motion = new updateMotionTask();
    }
    @AfterEach void cleanup() throws Exception {
        sync.clearFlightState();
        if (sync.movementService != null) {
            sync.movementService.shutdownNow();
            assertTrue(sync.movementService.awaitTermination(2, TimeUnit.SECONDS));
        }
        bot("session", null);
        bot("running", false);
    }
    static void bot(String name, Object value) throws Exception {
        Field f = Bot.class.getDeclaredField(name); f.setAccessible(true); f.set(Bot.INSTANCE, value);
    }
    long uses() { return packets.stream().filter(ServerboundUseItemPacket.class::isInstance).count(); }
    boolean fallbackActive() throws Exception {
        Field controls = MovementSync.class.getDeclaredField("flightControls");
        controls.setAccessible(true);
        Field active = FlightControlAdmission.class.getDeclaredField("fallbackActive");
        active.setAccessible(true);
        return active.getBoolean(controls.get(sync));
    }
    int fallbackRecoveryTicks() throws Exception {
        Field controls = MovementSync.class.getDeclaredField("flightControls");
        controls.setAccessible(true);
        Field ticks = FlightControlAdmission.class.getDeclaredField("fallbackRecoveryTicks");
        ticks.setAccessible(true);
        return ticks.getInt(controls.get(sync));
    }
    int fallbackTicks() throws Exception {
        Field controls = MovementSync.class.getDeclaredField("flightControls");
        controls.setAccessible(true);
        Field ticks = FlightControlAdmission.class.getDeclaredField("fallbackTicks");
        ticks.setAccessible(true);
        return ticks.getInt(controls.get(sync));
    }
    void attach() {
        Entity rocket = new Entity(42, UUID.randomUUID(), EntityType.FIREWORK_ROCKET,
                .5, 10, .5, 0, 0, 0, new Vector3d());
        rocket.getMetadata().put("attached_to_target", OptionalInt.of(sync.entityId));
        sync.getWorld().getEntities().put(42, rocket);
    }
    @Test void attachedFallbackStillRequiresSafeUnpoweredContinuation() throws Exception {
        lava = true;
        attach();
        var sweep = FlightControlAdmission.class.getDeclaredMethod("fallbackSweep",
                MovementSync.class, PhysicsState.class, float.class, float.class, boolean.class);
        sweep.setAccessible(true);
        PhysicsState witness = null;
        search:
        for (double y = 7.1; y <= 18; y += .1) {
            for (double vertical = -.4; vertical <= 0; vertical += .05) {
                for (double horizontal = 0; horizontal <= .8; horizontal += .1) {
                    var candidate = new PhysicsState(new Vector3d(.5, y, .5),
                            new Vector3d(0, vertical, horizontal), false, 0, PlayerPose.GLIDING);
                    Object unpowered = sweep.invoke(null, sync, candidate, 0f, -55f, false);
                    Object powered = sweep.invoke(null, sync, candidate, 0f, -55f, true);
                    if (unpowered.toString().equals("BLOCKED") && powered.toString().equals("SAFE")) {
                        witness = candidate;
                        break search;
                    }
                }
            }
        }
        assertNotNull(witness, "fixture must distinguish attachment expiry from continued thrust");
        sync.position.set(witness.position());
        sync.velocity.set(witness.velocity());
        sync.pitch.set(17f);

        sync.snapshotInput();

        assertFalse(fallbackActive(),
                "an attached snapshot cannot replace proof of the immediate unpowered future");
        assertEquals(17f, sync.pitch.get());
        assertNull(sync.pollBeforeMovementPacket());
    }

    @Test void routeAbsentRequestsAndConfirmedAttachmentClimbs() {
        lava = true;
        motion.run();
        assertEquals(1, uses(), "unowned exposed descent must receive a checked request on first tick");
        assertFalse(sync.hasAttachedFirework(), "request must not manufacture attachment");
        assertTrue(sync.velocity.get().y < 0, "unconfirmed request must not manufacture thrust");
        attach();
        for (int i = 0; i < 8; i++) motion.run();
        assertTrue(sync.velocity.get().y > 0);
        assertTrue(sync.position.get().y > 10);
        assertEquals(1, uses());
    }

    @Test void terminalNavigationFailureSuppressesOwnerlessHazardTakeover() throws Exception {
        lava = true;
        sync.abandonNavigationRequest(sync.getNavigationGeneration(), "flight leg failed");

        assertEquals(PlayerInput.NONE, sync.snapshotInput());
        assertNull(sync.pollBeforeMovementPacket(),
                "terminal failure must not leave a queued fallback action");
        assertEquals(0, uses(), "terminal failure must not dispatch a fallback rocket");
        assertEquals(0f, sync.pitch.get(), "terminal failure must leave natural descent unsteered");
        assertFalse(fallbackActive());
    }

    @Test void aNewNavigationRequestRestoresOrdinaryOwnerlessHazardFallback() throws Exception {
        lava = true;
        sync.abandonNavigationRequest(sync.getNavigationGeneration(), "flight leg failed");
        sync.beginNavigationRequest();

        sync.snapshotInput();

        assertEquals(-55f, sync.pitch.get(),
                "terminal suppression must be scoped to the failed generation");
        assertNotNull(sync.pollBeforeMovementPacket(),
                "a new request may again queue the checked ownerless fallback action");
        assertTrue(fallbackActive());
    }

    @Test void staleTerminalFailureCannotSuppressTheCurrentRequest() throws Exception {
        lava = true;
        long staleGeneration = sync.getNavigationGeneration();
        sync.beginNavigationRequest();

        sync.abandonNavigationRequest(staleGeneration, "late stale failure");
        sync.snapshotInput();

        assertFalse(sync.requiresNaturalDescent());
        assertEquals(-55f, sync.pitch.get());
        assertNotNull(sync.pollBeforeMovementPacket());
        assertTrue(fallbackActive());
    }

    @Test void newMetadataSessionDoesNotInheritTerminalDescentSuppression() {
        sync.abandonNavigationRequest(sync.getNavigationGeneration(), "flight leg failed");
        assertTrue(sync.requiresNaturalDescent());

        sync.resetFlightMetadataSession();

        assertFalse(sync.requiresNaturalDescent(),
                "a reconnect may legitimately resume an airborne recovery in the new session");
    }

    @Test void deathGenerationDoesNotInheritTerminalDescentSuppression() {
        sync.abandonNavigationRequest(sync.getNavigationGeneration(), "flight leg failed");
        assertTrue(sync.requiresNaturalDescent());

        sync.onPlayerDeath();

        assertFalse(sync.requiresNaturalDescent());
    }

    @Test void activeFallbackIsRetiredAndCannotLatchTheNextRequest() throws Exception {
        lava = true;
        sync.snapshotInput();
        assertTrue(fallbackActive());

        sync.abandonNavigationRequest(sync.getNavigationGeneration(), "flight leg failed");
        assertFalse(fallbackActive(), "terminal failure must retire the old physical fallback owner");
        assertNull(sync.pollBeforeMovementPacket(), "its queued rocket must be discarded");

        sync.beginNavigationRequest();
        sync.snapshotInput();
        assertTrue(fallbackActive());
        assertNotNull(sync.pollBeforeMovementPacket(),
                "the new request must not inherit the canceled fallback's request latch");
    }

    @Test void explicitCancelClaimsIndependentLandingBeforeItsFirstTick() throws Exception {
        lava = true;
        sync.movementController.pause();

        sync.cancelNavigation();
        sync.snapshotInput();

        assertFalse(fallbackActive(), "generic ownerless fallback must not preempt explicit landing");
        assertNull(sync.pollBeforeMovementPacket(), "the pre-tick landing handoff must not queue a fallback rocket");
        assertEquals(1, sync.movementController.getMovements().size());
        assertInstanceOf(xin.bbtt.movements.ElytraFlyMovement.class,
                sync.movementController.getMovements().get(0));
    }

    @Test void explicitCancelPreclaimsLandingBeforeBlockingOnStop() throws Exception {
        lava = true;
        CountDownLatch stopEntered = new CountDownLatch(1);
        CountDownLatch releaseStop = new CountDownLatch(1);
        sync.movementController.addMovement(new Movement() {
            @Override public void init() {}
            @Override public void onTick() {}
            @Override public long getTime() { return -1; }
            @Override public void onStop() {
                stopEntered.countDown();
                try {
                    if (!releaseStop.await(2, TimeUnit.SECONDS))
                        throw new AssertionError("timed out waiting to release stop");
                } catch (InterruptedException interrupted) {
                    Thread.currentThread().interrupt();
                    throw new AssertionError(interrupted);
                }
            }
        });
        CompletableFuture<Void> cancel = CompletableFuture.runAsync(sync::cancelNavigation);
        assertTrue(stopEntered.await(2, TimeUnit.SECONDS));
        try {
            sync.snapshotInput();
            assertFalse(fallbackActive(),
                    "generic fallback must not act while the canceled movement is still stopping");
            assertNull(sync.pollBeforeMovementPacket());
        } finally {
            releaseStop.countDown();
        }
        cancel.get(2, TimeUnit.SECONDS);
    }

    @Test void terminalFailureRestoresAttitudeBeforeNaturalDescent() throws Exception {
        lava = true;
        sync.snapshotInput();
        assertTrue(fallbackActive());
        assertEquals(-55f, sync.pitch.get());

        sync.abandonNavigationRequest(sync.getNavigationGeneration(), "flight leg failed");
        sync.snapshotInput();

        assertFalse(fallbackActive());
        assertEquals(0f, sync.pitch.get(),
                "terminal failure must not retain the fallback climb attitude during natural descent");
        assertNull(sync.pollBeforeMovementPacket());
    }
    @Test void invalidationCannotCreateAnUnownedHazardWindowOrResetCooldown() {
        lava = true;
        sync.claimFlightControlOwner(this, FlightControlAdmission.Mode.LOCAL_SOLVER);
        sync.invalidateFlightControls();
        for (int i = 0; i < 12; i++) { sync.invalidateFlightControls(); motion.run(); }
        assertEquals(1, uses());
        assertTrue(sync.pitch.get() < 0);
    }
    @Test void noRocketsStillGetsCheckedAttitudeButNoInventedClimb() {
        lava = true;
        sync.inventoryManager.getInventory()[36] = null;
        motion.run();
        assertTrue(sync.pitch.get() < 0);
        assertEquals(0, uses());
        assertTrue(sync.velocity.get().y < 0);
    }
    @Test void exposedLavaRequestsBeforeContact() { lava = true; motion.run(); assertEquals(1, uses()); }
    @Test void unknownTerrainCannotAuthorizeBoost() { lava = true; unknown = true; motion.run(); assertEquals(0, uses()); }
    @Test void priorRequestCooldownSurvivesOwnerHandoff() {
        lava = true;
        assertTrue(sync.useFirework());
        sync.invalidateFlightControls(); motion.run();
        assertEquals(1, uses());
    }
    @Test void metadataStopDoesNotGetOverridden() {
        sync.setGliding(false); motion.run(); assertEquals(0, uses()); assertFalse(sync.isGliding());
    }
    @Test void poweredCeilingIsRejected() { lava = true; ceiling = true; motion.run(); assertEquals(0, uses()); }
    @Test void supportedFlightUnchanged() { supported = true; motion.run(); assertEquals(0f, sync.pitch.get()); assertEquals(0, uses()); }
    @Test void expiredOwnedPrefixFallsBackInTheSameMotionTick() {
        lava = true;
        sync.claimFlightControlOwner(this, FlightControlAdmission.Mode.DIRECT);
        var source = FlightControlAdmission.snapshot(sync);
        var control = new ElytraFlightController.Decision(0, -5, false, ElytraFlightController.Phase.CRUISE);
        sync.submitFlightControls(new FlightControlAdmission.Envelope(this, source,
                List.of(new FlightControlAdmission.Step(control, source.state())), null, () -> {}, List.of()));
        motion.run(); assertEquals(0, uses());
        motion.run(); assertEquals(1, uses()); assertEquals(-55f, sync.pitch.get());
    }
    @Test void highEmptyAirRetainsExternalControlWithoutInventingHazardOwner() {
        sync.pitch.set(23f);
        motion.run();
        assertEquals(23f, sync.pitch.get());
        assertEquals(0, uses());
        assertNull(sync.getFlightControlFailure());
    }
    @Test void loadedVoidNearWorldBottomStillGetsFallback() {
        sync.position.set(new Vector3d(.5, World.getMinWorldY() + 10, .5));
        motion.run();
        assertEquals(-55f, sync.pitch.get());
        assertEquals(1, uses());
    }
    @Test void healthyUnownedVoidClimbIsNotOverridden() {
        sync.velocity.set(new Vector3d(0, .2, 1.5));
        motion.run(); assertEquals(0, uses()); assertEquals(0f, sync.pitch.get());
    }
    @Test void activeLowSlowFallbackDoesNotAlternateWithBriefLocalPrefixes() {
        lava = true;
        sync.claimFlightControlOwner(this, FlightControlAdmission.Mode.LOCAL_SOLVER);
        motion.run();
        assertEquals(-55f, sync.pitch.get());
        assertEquals(1, uses());

        var local = new ElytraFlightController.Decision(90, -5, false,
                ElytraFlightController.Phase.CRUISE);
        for (int tick = 0; tick < 2; tick++) {
            var source = FlightControlAdmission.snapshot(sync);
            assertTrue(sync.publishFlightPrefix(this, source,
                    List.of(new FlightControlAdmission.Step(local, source.state())),
                    List.of(), null, () -> {}));
            motion.run();
            assertEquals(-55f, sync.pitch.get(),
                    "one bounded hazard episode must not alternate with a momentary local owner");
            assertEquals(0f, sync.yaw.get() % 360f, .001f,
                    "fallback must retain the checked episode heading");
        }
        assertEquals(1, uses(), "ownership churn must not re-arm the physical request");

        attach();
        for (int recoveryTick = 0; recoveryTick < 3; recoveryTick++) {
            sync.velocity.set(new Vector3d(0, .2, .3));
            var recovered = FlightControlAdmission.snapshot(sync);
            assertTrue(sync.publishFlightPrefix(this, recovered,
                    List.of(new FlightControlAdmission.Step(local, recovered.state())),
                    List.of(), null, () -> {}));
            motion.run();
            assertEquals(-55f, sync.pitch.get());
        }
        sync.velocity.set(new Vector3d(0, -.08, 0));
        var interrupted = FlightControlAdmission.snapshot(sync);
        assertTrue(sync.publishFlightPrefix(this, interrupted,
                List.of(new FlightControlAdmission.Step(local, interrupted.state())),
                List.of(), null, () -> {}));
        motion.run();
        assertEquals(-55f, sync.pitch.get(), "a low/slow sample resets recovery confirmation");

        for (int recoveryTick = 0; recoveryTick < 4; recoveryTick++) {
            sync.velocity.set(new Vector3d(0, .2, .3));
            var recovered = FlightControlAdmission.snapshot(sync);
            assertTrue(sync.publishFlightPrefix(this, recovered,
                    List.of(new FlightControlAdmission.Step(local, recovered.state())),
                    List.of(), null, () -> {}));
            motion.run();
            if (recoveryTick < 3) assertEquals(-55f, sync.pitch.get(),
                    "one rising sample must not restart low/slow ownership churn");
            else assertEquals(-5f, sync.pitch.get(),
                    "sustained confirmed boost releases the bounded fallback");
        }
    }

    @Test void activeFallbackDoesNotAcceptSuppressedPrefixCallback() throws Exception {
        lava = true;
        sync.claimFlightControlOwner(this, FlightControlAdmission.Mode.LOCAL_SOLVER);
        motion.run();
        assertTrue(fallbackActive());
        long requests = uses();

        var accepted = new java.util.concurrent.atomic.AtomicInteger();
        var source = FlightControlAdmission.snapshot(sync);
        var local = new ElytraFlightController.Decision(90, -5, true,
                ElytraFlightController.Phase.CRUISE);
        assertTrue(sync.publishFlightPrefix(this, source,
                List.of(new FlightControlAdmission.Step(local, source.state())),
                List.of(), null, accepted::incrementAndGet));
        motion.run();

        assertEquals(0, accepted.get(), "a suppressed prefix must have no action side effect");
        assertEquals(requests, uses(), "fallback ownership must not dispatch its hidden rocket");
        assertTrue(fallbackActive());
        assertEquals(-55f, sync.pitch.get());
    }

    @Test void activeFallbackDoesNotReleaseOnOneUnknownTerrainSample() throws Exception {
        lava = true;
        sync.claimFlightControlOwner(this, FlightControlAdmission.Mode.LOCAL_SOLVER);
        motion.run();
        assertTrue(fallbackActive());
        assertEquals(-55f, sync.pitch.get());

        unknown = true;
        sync.snapshotInput();
        assertTrue(fallbackActive(), "unknown terrain is not sustained physical recovery");
        assertEquals(-55f, sync.pitch.get(),
                "an out-of-band observation must not alternate the active fallback heading");
    }

    @Test void activeFallbackReleasesWhenHazardBandIsKnownClear() throws Exception {
        lava = true;
        sync.claimFlightControlOwner(this, FlightControlAdmission.Mode.LOCAL_SOLVER);
        motion.run();
        assertTrue(fallbackActive());

        lava = false;
        sync.snapshotInput();
        assertFalse(fallbackActive(), "known clear terrain may release the bounded hazard owner");
    }

    @Test void repeatedInputSnapshotsCannotSatisfyFourPhysicalTickRecovery() throws Exception {
        lava = true;
        sync.claimFlightControlOwner(this, FlightControlAdmission.Mode.LOCAL_SOLVER);
        motion.run();
        assertTrue(fallbackActive());

        sync.velocity.set(new Vector3d(1, .2, 0));
        long physicsTick = sync.physicsTickVersion();
        for (int call = 0; call < 4; call++) sync.snapshotInput();
        assertEquals(physicsTick, sync.physicsTickVersion());
        assertTrue(fallbackActive(),
                "four input-boundary calls in one physical tick are only one recovery sample");
        assertEquals(1, fallbackRecoveryTicks());
        assertEquals(-55f, sync.pitch.get());
    }

    @Test void fallbackBudgetCountsCommittedPhysicsTicksNotInputCalls() throws Exception {
        lava = true;
        sync.claimFlightControlOwner(this, FlightControlAdmission.Mode.LOCAL_SOLVER);
        motion.run();
        assertTrue(fallbackActive());

        assertEquals(1, fallbackTicks());
        for (int call = 0; call < 30; call++) sync.snapshotInput();
        assertTrue(fallbackActive(), "duplicate input observations must not exhaust 24 attitude ticks");
        assertEquals(1, fallbackTicks(),
                "an uncommitted input boundary must not consume a physical attitude tick");

        // Acquisition used one boundary and the first duplicate call identified the next
        // physical boundary; advancing 22 more versions reaches 24 distinct attitudes.
        for (int tick = 0; tick < 22; tick++) {
            sync.completePhysicsStep();
            sync.snapshotInput();
            assertTrue(fallbackActive(), "episode released before 24 physical boundaries");
        }
        sync.completePhysicsStep();
        sync.snapshotInput();
        assertFalse(fallbackActive(), "the bounded episode must release after its committed budget");
    }

    @Test void unknownTerrainCannotExtendFallbackBeyondItsPhysicalBudget() throws Exception {
        lava = true;
        sync.claimFlightControlOwner(this, FlightControlAdmission.Mode.LOCAL_SOLVER);
        motion.run();
        assertTrue(fallbackActive());
        unknown = true;

        for (int tick = 0; tick < 23; tick++) {
            sync.completePhysicsStep();
            sync.snapshotInput();
            assertTrue(fallbackActive(), "unknown terrain must retain but not replenish the episode");
        }
        sync.completePhysicsStep();
        sync.snapshotInput();
        assertFalse(fallbackActive(), "fail-closed ownership must remain physically bounded");
    }

    @Test void admittedOwnerIsNotOverridden() {
        lava = true;
        sync.claimFlightControlOwner(this, FlightControlAdmission.Mode.DIRECT);
        var source = FlightControlAdmission.snapshot(sync);
        var control = new ElytraFlightController.Decision(0, -5, false, ElytraFlightController.Phase.CRUISE);
        sync.submitFlightControls(new FlightControlAdmission.Envelope(this, source,
                List.of(new FlightControlAdmission.Step(control, source.state())), null, () -> {}, List.of()));
        motion.run();
        assertEquals(-5f, sync.pitch.get()); assertEquals(0, uses());
    }
}
