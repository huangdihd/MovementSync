package xin.bbtt.physics;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.OptionalInt;
import java.util.UUID;
import java.util.concurrent.atomic.AtomicInteger;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import xin.bbtt.Entity.Entity;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

class KnownReturnFallMarginAdmissionTest extends ElytraCommandTestSupport {
    private final FlightControlAdmission admission = new FlightControlAdmission();
    private final Object owner = new Object();

    private void setupFlight() throws Exception {
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
        });
        movementSync.position.set(new Vector3d(0, 100, 0));
        movementSync.velocity.set(new Vector3d(0, 0, .3));
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        admission.claim(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
    }

    private FlightControlAdmission.Source source() {
        var snapshot = FlightControlAdmission.snapshot(movementSync);
        return new FlightControlAdmission.Source(snapshot.tick(), snapshot.generation(), snapshot.state(),
                admission.epoch());
    }

    private ElytraFlightController.Decision cruise(float pitch, boolean firework) {
        return new ElytraFlightController.Decision(0, pitch, firework, ElytraFlightController.Phase.CRUISE);
    }

    private List<FlightControlAdmission.Step> steps(float pitch) {
        return List.of(new FlightControlAdmission.Step(cruise(pitch, false), source().state()));
    }

    /** Choose the incoming distance so the next airborne result hits equality exactly. */
    private void setThresholdState(float pitch) {
        movementSync.velocity.set(new Vector3d(0, -.5, .3));
        movementSync.fallDistance.set(0d);
        var result = simulate(cruise(pitch, false), false);
        movementSync.fallDistance.set(movementSync.getMovementParameters().safeFallDistance()
                - result.fallDistance());
        result = simulate(cruise(pitch, false), false);
        assertFalse(result.onGround());
        assertEquals(movementSync.getMovementParameters().safeFallDistance(), result.fallDistance());
        assertEquals(0d, FlightHazards.damage(movementSync.getWorld(), source().state(), result,
                DamageParameters.NONE, false).raw(), "airborne threshold is not contact damage");
    }

    private PhysicsResult simulate(ElytraFlightController.Decision control, boolean powered) {
        return new GlidingPhysics(control.yaw(), control.pitch(), powered, PlayerInput.NONE,
                movementSync.getMovementParameters()).step(movementSync.getWorld(), source().state(),
                        PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
    }

    private void applyCurrentControl() {
        var result = simulate(cruise(movementSync.pitch.get(), false), movementSync.hasAttachedFirework());
        movementSync.position.set(result.position());
        movementSync.velocity.set(result.velocity());
        movementSync.onGround.set(result.onGround());
        movementSync.fallDistance.set(result.fallDistance());
        movementSync.completePhysicsStep();
    }

    @Test void existingConstructorsAndFactoriesDefaultToOrdinaryContactSafety() throws Exception {
        setupFlight();
        var checkpoint = source();
        var prefix = steps(0);
        assertFalse(new FlightControlAdmission.Envelope(owner, checkpoint, prefix, null, () -> {},
                List.of()).preserveFallMargin());
        assertFalse(new FlightControlAdmission.Envelope(owner, checkpoint, prefix, null, () -> {},
                List.of(), null).preserveFallMargin());
        assertFalse(new FlightControlAdmission.Envelope(owner, checkpoint, prefix, null, () -> {},
                List.of(), null, true).preserveFallMargin());
        assertFalse(new FlightControlAdmission.Envelope(owner, checkpoint, prefix, null, () -> {},
                List.of(), List.of(), null, true).preserveFallMargin());
        assertFalse(new FlightControlAdmission.Envelope(owner, checkpoint, prefix, null, () -> {},
                List.of(), List.of(), null, true, null).preserveFallMargin());
        assertFalse(new FlightControlAdmission.Envelope(owner, checkpoint, prefix, null, () -> {},
                List.of(), List.of(), null, true, null, null).preserveFallMargin());
        assertFalse(FlightControlAdmission.createPrefix(owner, checkpoint, prefix, List.of(), List.of(),
                null, () -> {}, true, null).preserveFallMargin());
    }

    @ParameterizedTest @ValueSource(booleans = {false, true})
    void liveReplayRejectsEqualityOnlyWithExplicitPolicy(boolean preserveFallMargin) throws Exception {
        setupFlight();
        var callback = new AtomicInteger();
        var envelope = FlightControlAdmission.createPrefix(owner, source(), steps(0), List.of(), List.of(),
                null, callback::incrementAndGet, true, null, preserveFallMargin);
        setThresholdState(0);
        movementSync.completePhysicsStep();
        admission.submit(envelope);
        admission.consume(movementSync);
        assertEquals(!preserveFallMargin, admission.ownershipTrace().active());
        assertEquals(preserveFallMargin ? 0 : 1, callback.get());
    }

    @ParameterizedTest @ValueSource(booleans = {false, true})
    void publicPublicationForwardsPolicyToLiveReplay(boolean preserveFallMargin) throws Exception {
        setupFlight();
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        var checkpoint = FlightControlAdmission.snapshot(movementSync);
        var callback = new AtomicInteger();
        assertTrue(movementSync.publishFlightPrefix(owner, checkpoint, steps(0), List.of(), List.of(),
                null, callback::incrementAndGet, true, null, preserveFallMargin));
        setThresholdState(0);
        movementSync.snapshotInput();
        assertEquals(!preserveFallMargin, movementSync.flightControlOwnership().active());
        assertEquals(preserveFallMargin ? 0 : 1, callback.get());
    }

    @ParameterizedTest @ValueSource(booleans = {false, true})
    void rejectedPrefixReserveRetainsPolicy(boolean preserveFallMargin) throws Exception {
        setupFlight();
        var callback = new AtomicInteger();
        var envelope = FlightControlAdmission.createPrefix(owner, source(), List.of(), steps(0), List.of(),
                null, callback::incrementAndGet, true, null, preserveFallMargin);
        setThresholdState(0);
        admission.submit(envelope);
        admission.consume(movementSync);
        assertEquals(!preserveFallMargin, admission.ownershipTrace().active());
        if (!preserveFallMargin) assertTrue(admission.ownershipTrace().recovering());
        assertEquals(0, callback.get(), "reserve cannot dispatch the rejected prefix action");
    }

    @ParameterizedTest @ValueSource(booleans = {false, true})
    void activeReserveRetainsPolicyAtHandoff(boolean preserveFallMargin) throws Exception {
        setupFlight();
        var envelope = FlightControlAdmission.createPrefix(owner, source(), steps(0), steps(0), List.of(),
                null, () -> {}, true, null, preserveFallMargin);
        admission.submit(envelope);
        admission.consume(movementSync);
        assertTrue(admission.ownershipTrace().active());
        setThresholdState(0);
        movementSync.completePhysicsStep();
        admission.consume(movementSync);
        assertEquals(!preserveFallMargin, admission.ownershipTrace().active());
        if (!preserveFallMargin) assertTrue(admission.ownershipTrace().recovering());
    }

    private FlightControlAdmission.Envelope lifetimeEnvelope(boolean preserveFallMargin,
            AtomicInteger callback) {
        var checkpoint = source();
        var lifetime = FireworkSchedule.lifetimeBounds(1);
        int prefixTicks = lifetime.maximumPoweredTicks() + 1 - 6;
        var prefix = new ArrayList<FlightControlAdmission.Step>();
        prefix.add(new FlightControlAdmission.Step(cruise(-20, true), checkpoint.state()));
        for (int tick = 1; tick < prefixTicks; tick++)
            prefix.add(new FlightControlAdmission.Step(cruise(-20, false), checkpoint.state()));
        var reserve = Collections.nCopies(6,
                new FlightControlAdmission.Step(cruise(-20, false), checkpoint.state()));
        var goal = new Vector3d(checkpoint.state().position()).add(0, 0, 32);
        var route = new WaypointFlightTarget(List.of(checkpoint.state().position(), goal),
                goal, false, false, 64);
        return FlightControlAdmission.createPrefix(owner, checkpoint, prefix, reserve, steps(-20), route,
                callback::incrementAndGet, true, lifetime, preserveFallMargin);
    }

    private void attachRocket() {
        var rocket = new Entity(9001, new UUID(0, 9001), EntityType.FIREWORK_ROCKET,
                0, 100, 0, 0, 0, 0, new Vector3d());
        rocket.getMetadata().put("attached_to_target", OptionalInt.of(movementSync.entityId));
        movementSync.getWorld().getEntities().put(9001, rocket);
    }

    @ParameterizedTest @ValueSource(booleans = {false, true})
    void boundedLifetimeAdmissionRejectsAirborneThreshold(boolean preserveFallMargin) throws Exception {
        setupFlight();
        var callback = new AtomicInteger();
        var envelope = lifetimeEnvelope(preserveFallMargin, callback);
        setThresholdState(-20);
        admission.submit(envelope);
        admission.consume(movementSync);
        assertEquals(!preserveFallMargin, admission.ownershipTrace().active());
        assertEquals(preserveFallMargin ? 0 : 1, callback.get());
    }

    @ParameterizedTest @ValueSource(booleans = {false, true})
    void admittedRocketLeaseRechecksThresholdFromLiveState(boolean preserveFallMargin) throws Exception {
        setupFlight();
        var callback = new AtomicInteger();
        admission.submit(lifetimeEnvelope(preserveFallMargin, callback));
        admission.consume(movementSync);
        assertTrue(admission.ownershipTrace().active());
        attachRocket();
        for (int tick = 0; tick < 5; tick++) {
            applyCurrentControl();
            admission.consume(movementSync);
            assertTrue(admission.ownershipTrace().active());
        }
        setThresholdState(-20);
        movementSync.completePhysicsStep();
        admission.consume(movementSync);
        assertEquals(!preserveFallMargin, admission.ownershipTrace().active());
        assertEquals(1, callback.get(), "live rejection cannot repeat the already dispatched resource action");
    }

    @Test void strictPolicyKeepsRequestRelativeClockAcrossDelayedAttachmentAndReserve() throws Exception {
        setupFlight();
        var callback = new AtomicInteger();
        admission.submit(lifetimeEnvelope(true, callback));
        admission.consume(movementSync);
        assertEquals(1, callback.get());
        for (int tick = 0; tick < 32; tick++) {
            if (tick == 2) attachRocket();
            applyCurrentControl();
            if (tick == 31) movementSync.getWorld().getEntities().remove(9001);
            admission.consume(movementSync);
            assertTrue(admission.ownershipTrace().active(), "request-relative lease at tick=" + (tick + 1));
            assertEquals(tick + 1 >= 27, admission.ownershipTrace().recovering());
            assertEquals(1, callback.get());
        }
    }

    @Test void fixedReturnReplayStopsAtThresholdBeforeContinuingResourceTimeline() throws Exception {
        setupFlight();
        setThresholdState(0);
        var initial = source().state();
        var control = cruise(0, false);
        var schedule = FireworkSchedule.confirmed(new int[0], 0, 7, 0, 0);
        var first = new KnownReturnPlan.Step(initial, FlightSafety.nextState(simulate(control, false)),
                control, schedule.nextTick());
        var second = new KnownReturnPlan.Step(first.after(), first.after(), control, schedule.nextTick());
        var plan = new KnownReturnPlan.Result(List.of(first, second), 2, "Equality before continuation");
        var replay = KnownReturnPlan.replay(movementSync.getWorld(), initial,
                movementSync.getMovementParameters(), plan);
        assertFalse(replay.proven());
        assertEquals(1, replay.simulatedSteps(), "the equality step must fail before replaying the next resource tick");
        assertEquals(6, first.resource().checkpoint().cooldownTicks());
        assertEquals(5, second.resource().checkpoint().cooldownTicks());
    }

    @Test void directReturnWitnessCannotAuthorizeThresholdEquality() throws Exception {
        setupFlight();
        setThresholdState(0);
        var checkpoint = source();
        var control = cruise(0, false);
        var resource = FireworkSchedule.confirmed(new int[0], 0, 0, 0, 0).nextTick();
        var step = new KnownReturnPlan.Step(checkpoint.state(), FlightSafety.nextState(simulate(control, false)),
                control, resource);
        var witness = new KnownReturnPlan.Result(List.of(step), 1, "Airborne equality fixture");
        var envelope = FlightControlAdmission.knownReturn(owner, checkpoint, witness, new int[0], ignored ->
                fail("an unpowered rejected return must not dispatch an action"));
        assertTrue(envelope.preserveFallMargin());
        admission.submit(envelope);
        admission.consume(movementSync);
        assertFalse(admission.ownershipTrace().active());
        assertNotNull(admission.failure());
    }
}
