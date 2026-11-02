package xin.bbtt.physics;

import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.flight.LocalClimbHandoff;
import xin.bbtt.flight.LocalGotoController;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

class FlightControlPrefixAdmissionTest extends ElytraCommandTestSupport {
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
        return new FlightControlAdmission.Source(snapshot.tick(), snapshot.generation(), snapshot.state(), admission.epoch());
    }

    private List<FlightControlAdmission.Step> steps(float yaw, float pitch) {
        return List.of(new FlightControlAdmission.Step(new ElytraFlightController.Decision(yaw, pitch,
                false, ElytraFlightController.Phase.CRUISE), source().state()));
    }

    private WaypointFlightTarget wideForwardRoute(FlightControlAdmission.Source source) {
        var goal = new Vector3d(source.state().position()).add(0, 0, 32);
        return new WaypointFlightTarget(List.of(source.state().position(), goal),
                goal, false, false, 64);
    }

    private WaypointFlightTarget routeTo(FlightControlAdmission.Source source, double x, double z) {
        var goal = new Vector3d(source.state().position()).add(x, 0, z);
        return new WaypointFlightTarget(List.of(source.state().position(), goal),
                goal, false, false, 64);
    }

    @Test void delayedSourceWithCurrentAuthorityAcceptsDifferentExecutionWitness() throws Exception {
        setupFlight();
        synchronized (movementSync) {
            var source = source();
            var authorityRoute = wideForwardRoute(source);
            var executionRoute = routeTo(source, 24, 8);
            assertTrue(admission.registerForwardEmergency(movementSync, owner, source, 0, authorityRoute));
            movementSync.completePhysicsStep();
            assertTrue(admission.publishPrefix(movementSync, owner, source, steps(0, -5), List.of(),
                    List.of(), executionRoute, authorityRoute, () -> {}, false, null, false),
                    "a delayed worker may publish an execution witness under the current authority route");
            admission.consume(movementSync);
            assertEquals(0f, movementSync.yaw.get(), .001f,
                    "physical replay must validate and steer through the execution witness");
        }
    }

    @Test void authorityRouteChangeAndWrongRouteRejectPublication() throws Exception {
        setupFlight();
        synchronized (movementSync) {
            var source = source();
            var authorityRoute = wideForwardRoute(source);
            var differentRoute = routeTo(source, 32, 0);
            assertTrue(admission.registerForwardEmergency(movementSync, owner, source, 0, authorityRoute));
            assertFalse(admission.publishPrefix(movementSync, owner, source, steps(0, -5), List.of(),
                    List.of(), differentRoute, differentRoute, () -> {}, false, null, false));
            assertFalse(admission.publishPrefix(movementSync, owner, source, steps(0, -5), List.of(),
                    List.of(), differentRoute, routeTo(source, -32, 0), () -> {}, false, null, false));
        }
    }

    @Test void ownerEpochGenerationAndSourceAgeRejectPublication() throws Exception {
        setupFlight();
        synchronized (movementSync) {
            var source = source();
            var authorityRoute = wideForwardRoute(source);
            assertTrue(admission.registerForwardEmergency(movementSync, owner, source, 0, authorityRoute));

            var wrongGeneration = new FlightControlAdmission.Source(source.tick(),
                    source.generation() + 1, source.state(), source.epoch());
            assertFalse(admission.publishPrefix(movementSync, owner, wrongGeneration, steps(0, -5), List.of(),
                    List.of(), authorityRoute, authorityRoute, () -> {}, false, null, false));

            var otherOwner = new Object();
            admission.claim(otherOwner, FlightControlAdmission.Mode.LOCAL_SOLVER);
            assertFalse(admission.publishPrefix(movementSync, owner, source, steps(0, -5), List.of(),
                    List.of(), authorityRoute, authorityRoute, () -> {}, false, null, false));
        }

        setupFlight();
        synchronized (movementSync) {
            var source = source();
            var authorityRoute = wideForwardRoute(source);
            assertTrue(admission.registerForwardEmergency(movementSync, owner, source, 0, authorityRoute));
            for (int index = 0; index < 9; index++) movementSync.completePhysicsStep();
            assertFalse(admission.publishPrefix(movementSync, owner, source, steps(0, -5), List.of(),
                    List.of(), authorityRoute, authorityRoute, () -> {}, false, null, false));
        }
    }

    @Test void revokedAuthorityRouteRejectsLaterPublication() throws Exception {
        setupFlight();
        synchronized (movementSync) {
            var source = source();
            var authorityRoute = wideForwardRoute(source);
            assertTrue(admission.registerForwardEmergency(movementSync, owner, source, 0, authorityRoute));
            admission.invalidateForwardEmergency(owner);
            assertFalse(admission.publishPrefix(movementSync, owner, source, steps(0, -5), List.of(),
                    List.of(), authorityRoute, authorityRoute, () -> {}, false, null, false));
        }
    }

    @Test void slowWorkerExhaustsExactPrefixThenFixedRecoveryThenForwardEmergency() throws Exception {
        setupFlight();
        synchronized (movementSync) {
            var prefix = steps(0, 0);
            assertTrue(FlightControlAdmission.createPrefix(owner, source(), prefix, steps(180, 25),
                    null, () -> {}).recovery().isEmpty(), "reverse descent is never a braking reserve");
            var recovery = steps(0, -45);
            var envelope = FlightControlAdmission.createPrefix(owner, source(), prefix, recovery, null, () -> {});
            assertEquals(prefix, envelope.steps(), "no appended terminal rollout");
            assertEquals(recovery, envelope.recovery());
            assertTrue(envelope.handoff().isEmpty(), "legacy envelopes have no proof-only continuation");
            var emergencySource = source();
            assertTrue(admission.registerForwardEmergency(movementSync, owner, emergencySource, 0,
                    wideForwardRoute(emergencySource)));
            admission.submit(envelope);
            admission.consume(movementSync);
            assertTrue(admission.ownershipTrace().active());
            movementSync.completePhysicsStep();
            admission.consume(movementSync);
            assertTrue(admission.ownershipTrace().recovering());
            movementSync.completePhysicsStep();
            admission.consume(movementSync);
            assertFalse(admission.ownershipTrace().active());
            assertEquals(0f, movementSync.yaw.get() % 360f, .001f);
            assertEquals(-20f, movementSync.pitch.get());
            assertTrue(admission.failure().reason().startsWith("UNPROVEN"));
            assertNull(movementSync.pollBeforeMovementPacket());
            for (int i = 0; i < 12; i++) {
                movementSync.completePhysicsStep();
                admission.consume(movementSync);
                assertEquals(-20f, movementSync.pitch.get());
                assertEquals(0f, movementSync.yaw.get() % 360f, .001f);
            }
        }
    }

    @Test void shorterRenewalCannotReduceAStillValidExecutableHorizon() throws Exception {
        setupFlight();
        synchronized (movementSync) {
            var initial = source();
            var control = new ElytraFlightController.Decision(0, -20, false,
                    ElytraFlightController.Phase.CRUISE);
            var longPrefix = java.util.stream.IntStream.range(0, 10)
                    .mapToObj(index -> new FlightControlAdmission.Step(control, initial.state())).toList();
            var reserve = java.util.stream.IntStream.range(0, 6)
                    .mapToObj(index -> new FlightControlAdmission.Step(control, initial.state())).toList();
            admission.submit(FlightControlAdmission.createPrefix(owner, initial,
                    longPrefix, reserve, null, () -> {}));
            admission.consume(movementSync);
            assertEquals(initial.tick(), admission.ownershipTrace().sourceTick());

            movementSync.completePhysicsStep();
            var renewal = source();
            var shortPrefix = java.util.stream.IntStream.range(0, 3)
                    .mapToObj(index -> new FlightControlAdmission.Step(control, renewal.state())).toList();
            var callback = new AtomicInteger();
            admission.submit(FlightControlAdmission.createPrefix(owner, renewal,
                    shortPrefix, reserve, null, callback::incrementAndGet));
            admission.consume(movementSync);

            assertEquals(initial.tick(), admission.ownershipTrace().sourceTick(),
                    "a shorter renewal must not discard a longer live-revalidated commitment");
            assertEquals(1, admission.ownershipTrace().cursor());
            assertEquals(0, callback.get(), "a rejected renewal cannot dispatch its action");
        }
    }

    @Test void corridorBoundRecoveryRetainsProvenSteeringAtHandoff() throws Exception {
        setupFlight();
        synchronized (movementSync) {
            var source = source();
            var prefixControl = new ElytraFlightController.Decision(0, 0, false,
                    ElytraFlightController.Phase.CRUISE);
            var recoveryControl = new ElytraFlightController.Decision(180, 25, false,
                    ElytraFlightController.Phase.CRUISE);
            var prefix = List.of(new FlightControlAdmission.Step(prefixControl, source.state()));
            var recovery = List.of(new FlightControlAdmission.Step(recoveryControl, source.state()));
            var envelope = FlightControlAdmission.createPrefix(owner, source, prefix, recovery,
                    wideForwardRoute(source), () -> {});
            assertEquals(recovery, envelope.recovery(),
                    "a corridor-bound reserve has already proved its steering against that corridor");

            admission.submit(envelope);
            admission.consume(movementSync);
            movementSync.completePhysicsStep();
            admission.consume(movementSync);

            assertTrue(admission.ownershipTrace().recovering());
            assertEquals(180f, Math.abs(movementSync.yaw.get()), .001f);
            assertEquals(25f, movementSync.pitch.get());
        }
    }

    @Test void staleEpochAndGenerationCannotPublishOrReplaceForwardHeading() throws Exception {
        setupFlight();
        synchronized (movementSync) {
            var current = source();
            assertTrue(admission.registerForwardEmergency(movementSync, owner, current, 0,
                    wideForwardRoute(current)));
            for (var stale : List.of(
                    new FlightControlAdmission.Source(current.tick(), current.generation(), current.state(), current.epoch() - 1),
                    new FlightControlAdmission.Source(current.tick(), current.generation() - 1, current.state(), current.epoch()))) {
                assertFalse(admission.registerForwardEmergency(movementSync, owner, stale, 180));
                assertFalse(admission.publishPrefix(movementSync, owner, stale, steps(180, 25), List.of(), null, () -> fail("stale callback")));
            }
            admission.consume(movementSync);
            assertEquals(-20f, movementSync.pitch.get());
            assertEquals(0f, movementSync.yaw.get() % 360f, .001f);
        }
    }

    @Test void olderFreshWorkerCannotReplaceNewerEmergencyHeadingOrCorridor() throws Exception {
        setupFlight();
        synchronized (movementSync) {
            var older = source();
            movementSync.completePhysicsStep();
            var newer = source();
            var forwardGoal = new Vector3d(0, 100, 20);
            var forwardRoute = new WaypointFlightTarget(
                    List.of(newer.state().position(), forwardGoal), forwardGoal, false, false, 64);
            assertTrue(admission.registerForwardEmergency(movementSync, owner, newer, 0, forwardRoute));

            var staleGoal = new Vector3d(20, 100, 0);
            var staleRoute = new WaypointFlightTarget(
                    List.of(older.state().position(), staleGoal), staleGoal, false);
            assertFalse(admission.registerForwardEmergency(movementSync, owner, older, -90, staleRoute),
                    "an older but still fresh worker must not replace the atomic heading/corridor checkpoint");
            assertFalse(admission.publishPrefix(movementSync, owner, older,
                            List.of(new FlightControlAdmission.Step(
                                    new ElytraFlightController.Decision(-90, -5, false,
                                            ElytraFlightController.Phase.CRUISE), older.state())),
                            List.of(), staleRoute, () -> fail("stale callback")),
                    "a rejected old checkpoint must not publish controls beside the newer heading/corridor");

            admission.consume(movementSync);

            assertEquals(0f, movementSync.yaw.get() % 360f, .001f);
            assertEquals(-20f, movementSync.pitch.get());
        }
    }

    @Test void newerObservationOfSameCorridorDoesNotStarveFreshWorker() throws Exception {
        setupFlight();
        synchronized (movementSync) {
            var older = source();
            var route = wideForwardRoute(older);
            assertTrue(admission.registerForwardEmergency(movementSync, owner, older, 0, route));

            movementSync.completePhysicsStep();
            var newer = source();
            assertTrue(admission.registerForwardEmergency(movementSync, owner, newer, -90, route));
            assertTrue(admission.refineForwardEmergency(movementSync, owner, older, 45, route),
                    "a fresh worker for the same corridor must remain publishable");
            assertTrue(admission.publishPrefix(movementSync, owner, older, steps(0, 0), List.of(),
                    route, () -> {}));

            admission.consume(movementSync);
            assertTrue(admission.ownershipTrace().active());
            assertEquals(0f, movementSync.yaw.get(), .001f);

            movementSync.completePhysicsStep();
            admission.consume(movementSync);
            assertFalse(admission.ownershipTrace().active());
            assertEquals(-90f, movementSync.yaw.get(), .001f,
                    "the older worker must not overwrite the newer emergency heading");
            assertEquals(-20f, movementSync.pitch.get());
        }
    }

    @Test void sameTickNewerObservationOfSameCorridorDoesNotStarveFreshWorker() throws Exception {
        setupFlight();
        synchronized (movementSync) {
            var older = source();
            var route = wideForwardRoute(older);
            assertTrue(admission.registerForwardEmergency(movementSync, owner, older, 0, route));

            movementSync.position.set(new Vector3d(1, 100, 0));
            var newer = source();
            assertEquals(older.tick(), newer.tick());
            assertTrue(admission.registerForwardEmergency(movementSync, owner, newer, -90, route));

            assertTrue(admission.refineForwardEmergency(movementSync, owner, older, 45, route));
            assertTrue(admission.publishPrefix(movementSync, owner, older, steps(0, 0), List.of(),
                    route, () -> {}));
            admission.consume(movementSync);

            assertTrue(admission.ownershipTrace().active());
            assertEquals(0f, movementSync.yaw.get(), .001f);
        }
    }

    @Test void sameTickReplacementRejectsTheEarlierWorkerCheckpoint() throws Exception {
        setupFlight();
        synchronized (movementSync) {
            var older = source();
            var olderRoute = wideForwardRoute(older);
            assertTrue(admission.registerForwardEmergency(
                    movementSync, owner, older, 0, olderRoute));

            movementSync.position.set(new Vector3d(1, 100, 0));
            var newer = source();
            assertEquals(older.tick(), newer.tick());
            assertNotSame(older, newer);
            var newerGoal = new Vector3d(33, 100, 0);
            var newerRoute = new WaypointFlightTarget(
                    List.of(newer.state().position(), newerGoal), newerGoal, false, false, 64);
            assertTrue(admission.registerForwardEmergency(
                    movementSync, owner, newer, -90, newerRoute));

            assertFalse(admission.refineForwardEmergency(
                    movementSync, owner, older, 0, olderRoute));
            assertFalse(admission.publishPrefix(movementSync, owner, older,
                    List.of(new FlightControlAdmission.Step(
                            new ElytraFlightController.Decision(0, -5, false,
                                    ElytraFlightController.Phase.CRUISE), older.state())),
                    List.of(), olderRoute, () -> fail("stale callback")));
            assertTrue(admission.refineForwardEmergency(
                    movementSync, owner, newer, -90, newerRoute));
        }
    }

    @Test void invalidationClearsPendingAndRevokedSourceCannotReauthorizeItself() throws Exception {
        setupFlight();
        synchronized (movementSync) {
            var routeA = wideForwardRoute(source());
            var sourceA = source();
            var accepted = new AtomicInteger();
            assertTrue(admission.registerForwardEmergency(movementSync, owner, sourceA, 0, routeA));
            assertTrue(admission.publishPrefix(movementSync, owner, sourceA,
                    steps(0, -5), List.of(), routeA, accepted::incrementAndGet));

            admission.invalidateForwardEmergency(owner);

            assertFalse(admission.registerForwardEmergency(movementSync, owner, sourceA, 180, routeA),
                    "the revoked worker capability must not revive its own emergency checkpoint");
            assertFalse(admission.refineForwardEmergency(movementSync, owner, sourceA, 180, routeA));
            assertFalse(admission.publishPrefix(movementSync, owner, sourceA,
                    steps(180, 25), List.of(), routeA, accepted::incrementAndGet));
            admission.consume(movementSync);
            assertEquals(0, accepted.get(), "a revoked pending envelope must never dispatch its callback");
            assertFalse(admission.ownershipTrace().active());
        }
    }

    @Test void invalidationPreservesActivePrefixThenOnlyFreshRouteCanTakeOver() throws Exception {
        setupFlight();
        synchronized (movementSync) {
            var sourceA = source();
            var routeA = wideForwardRoute(sourceA);
            var controlA = new ElytraFlightController.Decision(0, -5, false,
                    ElytraFlightController.Phase.CRUISE);
            var prefixA = List.of(
                    new FlightControlAdmission.Step(controlA, sourceA.state()),
                    new FlightControlAdmission.Step(controlA, sourceA.state()));
            assertTrue(admission.registerForwardEmergency(movementSync, owner, sourceA, 0, routeA));
            assertTrue(admission.publishPrefix(movementSync, owner, sourceA,
                    prefixA, List.of(), routeA, () -> {}));
            admission.consume(movementSync);
            assertTrue(admission.ownershipTrace().active());

            admission.invalidateForwardEmergency(owner);
            movementSync.completePhysicsStep();
            admission.consume(movementSync);

            assertTrue(admission.ownershipTrace().active(),
                    "route invalidation must not tear down an already admitted finite prefix");
            assertEquals(-5f, movementSync.pitch.get());

            movementSync.completePhysicsStep();
            admission.consume(movementSync);
            assertFalse(admission.ownershipTrace().active());
            assertFalse(admission.registerForwardEmergency(movementSync, owner, sourceA, 180, routeA),
                    "exhaustion must not revive the revoked route-A fallback");

            var sourceB = source();
            var goalB = new Vector3d(32, 100, 0);
            var routeB = new WaypointFlightTarget(List.of(sourceB.state().position(), goalB),
                    goalB, false, false, 64);
            assertTrue(admission.registerForwardEmergency(movementSync, owner, sourceB, -90, routeB));
            assertTrue(admission.publishPrefix(movementSync, owner, sourceB,
                    steps(-90, -5), List.of(), routeB, () -> {}));
            admission.consume(movementSync);

            assertTrue(admission.ownershipTrace().active());
            assertEquals(-90f, movementSync.yaw.get(), .001f,
                    "only the fresh route-B checkpoint may take over after revocation");
        }
    }

    @Test void routeLessEmergencyRegistrationCannotAuthorizeSteering() throws Exception {
        setupFlight();
        synchronized (movementSync) {
            movementSync.pitch.set(23f);
            assertTrue(admission.registerForwardEmergency(movementSync, owner, source(), 0));

            admission.consume(movementSync);

            assertEquals(23f, movementSync.pitch.get(),
                    "a heading without its route corridor is diagnostic metadata, not steering permission");
            assertNotNull(admission.failure());
        }
    }

    @Test void exactLandingProofCannotLeaveAndReenterItsCorridor() throws Exception {
        var floor = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) { return minY < 10; }
            @Override public xin.bbtt.Block.BlockState getBlockStateAt(Vector3d p) {
                return p.y < 10
                        ? new xin.bbtt.Block.BlockState("minecraft:stone", 1,
                                java.util.Map.of(), "block", 1.5, true, "STONE")
                        : super.getBlockStateAt(p);
            }
        };
        var world = MovementSync.class.getDeclaredField("world");
        world.setAccessible(true);
        world.set(movementSync, floor);
        var initial = new PhysicsState(new Vector3d(.5, 12, .5),
                new Vector3d(.15, -.2, .4), false, 1, PlayerPose.GLIDING);
        var control = new ElytraFlightController.Decision(0, 85, false,
                ElytraFlightController.Phase.LANDING);
        PhysicsState state = initial;
        for (int i = 0; i < 80 && !state.onGround(); i++) {
            var result = new GlidingPhysics(control.yaw(), control.pitch(), false, PlayerInput.NONE,
                    movementSync.getMovementParameters()).step(floor, state,
                            PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            state = FlightSafety.nextState(result);
        }
        assertTrue(state.onGround(), "fixture must reach supported ground");
        var goal = state.position();
        var detour = new Vector3d(initial.position()).add(8, 0, 0);
        var corridor = new WaypointFlightTarget(
                List.of(initial.position(), detour, goal), goal, true, false, 0);
        var checkedLanding = FlightControlAdmission.class.getDeclaredMethod("checkedLanding",
                MovementSync.class, PhysicsState.class, ElytraFlightController.Decision.class,
                WaypointFlightTarget.class);
        checkedLanding.setAccessible(true);

        assertEquals(Boolean.TRUE, checkedLanding.invoke(null, movementSync, initial, control, null),
                "fixture must be an otherwise valid supported landing");
        assertEquals(Boolean.FALSE, checkedLanding.invoke(null, movementSync, initial, control, corridor),
                "terminal justification may not leave and later re-enter an exact execution corridor");
    }

    @Test void invalidPendingPreservesValidActivePrefix() throws Exception {
        setupFlight();
        synchronized (movementSync) {
            assertTrue(admission.publishPrefix(movementSync, owner, source(), steps(0, -5), List.of(), null, () -> {}));
            admission.consume(movementSync);
            long activation = admission.ownershipTrace().activation();
            assertTrue(admission.publishPrefix(movementSync, owner, source(), List.of(), List.of(), null, () -> fail("empty prefix")));
            admission.consume(movementSync);
            assertEquals(activation, admission.ownershipTrace().activation());
            assertTrue(admission.ownershipTrace().active());
            assertEquals(-5f, movementSync.pitch.get());
        }
    }

    @Test void movingPursuitCannotAdmitGroundedGlideMetadataRace() throws Exception {
        setupFlight();
        synchronized (movementSync) {
            var planned = source();
            var climb = new ElytraFlightController.Decision(0, -45, false,
                    ElytraFlightController.Phase.CRUISE);
            var envelope = FlightControlAdmission.createPrefix(owner, planned,
                    List.of(new FlightControlAdmission.Step(climb, planned.state())), List.of(), null,
                    () -> {}, true);
            movementSync.onGround.set(true);
            movementSync.velocity.set(new Vector3d(0, .2, .3));

            admission.submit(envelope);
            admission.consume(movementSync);

            assertFalse(admission.ownershipTrace().active(),
                    "moving pursuit must not launch from grounded glide metadata");
            assertNotNull(admission.failure());
        }
    }

    @Test void rejectedPrefixCannotEscapeItsCorridorThroughFixedRecovery() throws Exception {
        setupFlight();
        var planned = source();
        var prefix = new ElytraFlightController.Decision(0, -5, false,
                ElytraFlightController.Phase.CRUISE);
        var recovery = new ElytraFlightController.Decision(0, -45, false,
                ElytraFlightController.Phase.CRUISE);
        var route = new WaypointFlightTarget(List.of(planned.state().position(),
                new Vector3d(0, 100, 40)), new Vector3d(0, 100, 40), false);
        var envelope = FlightControlAdmission.createPrefix(owner, planned,
                List.of(new FlightControlAdmission.Step(prefix, planned.state())),
                List.of(new FlightControlAdmission.Step(recovery, planned.state())), route, () -> {});
        movementSync.position.set(new Vector3d(4, 100, 0));
        movementSync.pitch.set(33f);

        admission.submit(envelope);
        admission.consume(movementSync);

        assertEquals(33f, movementSync.pitch.get(),
                "fixed recovery must retain the rejected prefix's corridor");
        assertFalse(admission.ownershipTrace().active());
        assertNotNull(admission.failure());
    }

    @Test void terminalCorridorCannotAdmitSafeTouchdownOutsideItsGoal() throws Exception {
        setupFlight();
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) { return position.y < 1 ? 1 : 0; }
        });
        var planned = new PhysicsState(new Vector3d(.5, 1.02, .5), new Vector3d(0, -.2, .1), false,
                0, PlayerPose.GLIDING);
        movementSync.position.set(planned.position());
        movementSync.velocity.set(planned.velocity());
        movementSync.onGround.set(false);
        var source = new FlightControlAdmission.Source(movementSync.physicsTickVersion(),
                movementSync.getNavigationGeneration(), planned, admission.epoch());
        var control = new ElytraFlightController.Decision(0, 85, false,
                ElytraFlightController.Phase.LANDING);
        var result = new GlidingPhysics(control.yaw(), control.pitch(), false, PlayerInput.NONE,
                movementSync.getMovementParameters()).step(movementSync.getWorld(), planned,
                        PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
        assertTrue(result.onGround(), "fixture must touch safe loaded support on its first replay step");
        var wrongGoal = new Vector3d(20.5, 1, .5);
        var corridor = new WaypointFlightTarget(List.of(planned.position(), result.position()),
                wrongGoal, true, false, 0);
        var envelope = FlightControlAdmission.createPrefix(owner, source,
                List.of(new FlightControlAdmission.Step(control, planned)), List.of(), corridor, () -> {});
        movementSync.pitch.set(-5f);

        admission.submit(envelope);
        admission.consume(movementSync);

        assertEquals(-5f, movementSync.pitch.get(),
                "safe support outside the terminal goal must not authorize touchdown");
        assertFalse(admission.ownershipTrace().active());
        assertNotNull(admission.failure());
    }

    @Test void poweredTouchdownCannotDispatchFireworkEvenAtAcceptedGoal() throws Exception {
        setupFlight();
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) { return position.y < 1 ? 1 : 0; }
        });
        var planned = new PhysicsState(new Vector3d(.5, 1.02, .5), new Vector3d(0, -.2, .1), false,
                0, PlayerPose.GLIDING);
        movementSync.position.set(planned.position());
        movementSync.velocity.set(planned.velocity());
        movementSync.onGround.set(false);
        var source = new FlightControlAdmission.Source(movementSync.physicsTickVersion(),
                movementSync.getNavigationGeneration(), planned, admission.epoch());
        var control = new ElytraFlightController.Decision(0, 85, true,
                ElytraFlightController.Phase.LANDING);
        var powered = new GlidingPhysics(control.yaw(), control.pitch(), true, PlayerInput.NONE,
                movementSync.getMovementParameters()).step(movementSync.getWorld(), planned,
                        PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
        assertTrue(powered.onGround(), "fixture must expose a powered same-step touchdown");
        var corridor = new WaypointFlightTarget(List.of(planned.position(), powered.position()),
                powered.position(), true, false, 0);
        var accepted = new AtomicInteger();
        var envelope = FlightControlAdmission.createPrefix(owner, source,
                List.of(new FlightControlAdmission.Step(control, planned)), List.of(), corridor,
                accepted::incrementAndGet);

        admission.submit(envelope);
        admission.consume(movementSync);

        assertEquals(0, accepted.get(), "touchdown while simulated thrust is attached cannot dispatch a rocket");
    }

    @Test void rebasedReserveRejectsUnsafeHandoffBeforeApplyingItsDangerousTail() throws Exception {
        setupFlight();
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) { return position.x >= 1 ? 1 : 0; }
        });
        var cruise = new ElytraFlightController.Decision(0, 0, false,
                ElytraFlightController.Phase.CRUISE);
        var bridgeControl = new ElytraFlightController.Decision(-90, 0, false,
                ElytraFlightController.Phase.CRUISE);
        var plannedState = new PhysicsState(new Vector3d(-10, 100, 0),
                new Vector3d(0, 0, .3), false, 0, PlayerPose.GLIDING);
        var source = new FlightControlAdmission.Source(movementSync.physicsTickVersion(),
                movementSync.getNavigationGeneration(), plannedState, admission.epoch());
        var prefix = List.of(new FlightControlAdmission.Step(cruise, plannedState));
        var reserve = java.util.stream.IntStream.range(0, 6)
                .mapToObj(index -> new FlightControlAdmission.Step(cruise, plannedState)).toList();
        var handoff = List.of(new FlightControlAdmission.Step(bridgeControl, plannedState));
        var envelope = FlightControlAdmission.createPrefix(owner, source, prefix, reserve, handoff,
                null, () -> {}, true);
        var batch = TickControlBatch.admitted(envelope, 17);
        assertEquals(1, batch.commands().size(),
                "proof-only handoff and reserve must not extend the executable prefix batch");
        assertEquals(cruise.pitch(), batch.commands().get(0).pitch());

        PhysicsState plannedReplay = plannedState;
        for (var step : java.util.stream.Stream.of(prefix, reserve, handoff)
                .flatMap(List::stream).toList()) {
            var result = new GlidingPhysics(step.control().yaw(), step.control().pitch(), false,
                    PlayerInput.NONE, movementSync.getMovementParameters()).step(
                            movementSync.getWorld(), plannedReplay, PlayerPose.HALF_WIDTH,
                            PlayerPose.GLIDING.height());
            assertFalse(result.collidedX() || result.collidedZ(),
                    "the worker-side episode must be safe at its original source");
            plannedReplay = FlightSafety.nextState(result);
        }

        var liveState = new PhysicsState(new Vector3d(.699, 100, 0),
                new Vector3d(0, 0, .3), false, 0, PlayerPose.GLIDING);
        PhysicsState liveReplay = liveState;
        for (var step : java.util.stream.Stream.of(prefix, reserve).flatMap(List::stream).toList()) {
            var result = new GlidingPhysics(step.control().yaw(), step.control().pitch(), false,
                    PlayerInput.NONE, movementSync.getMovementParameters()).step(
                            movementSync.getWorld(), liveReplay, PlayerPose.HALF_WIDTH,
                            PlayerPose.GLIDING.height());
            assertFalse(result.collidedX() || result.collidedZ(),
                    "all executable controls must remain safe after rebasing");
            liveReplay = FlightSafety.nextState(result);
        }
        var bridgeResult = new GlidingPhysics(bridgeControl.yaw(), bridgeControl.pitch(), false,
                PlayerInput.NONE, movementSync.getMovementParameters()).step(movementSync.getWorld(),
                        liveReplay, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
        assertTrue(bridgeResult.collidedX(), "fixture handoff must hit the live wall");

        movementSync.position.set(liveState.position());
        movementSync.velocity.set(liveState.velocity());
        movementSync.onGround.set(false);
        admission.submit(envelope);
        admission.consume(movementSync);
        assertTrue(admission.ownershipTrace().active(),
                "the six-step window must not inspect a seventh control early");
        applyCurrentControl();

        admission.consume(movementSync);
        assertTrue(admission.ownershipTrace().recovering(),
                "the complete six-step reserve remains executable at handoff");
        applyCurrentControl();

        admission.consume(movementSync);
        assertFalse(admission.ownershipTrace().active(),
                "five remaining reserve steps plus the handoff must fail closed");
        assertEquals(0f, movementSync.yaw.get() % 360f, .001f,
                "the proof-only handoff must never reach the applied control");
        assertNotNull(admission.failure());
    }

    @Test void directEnvelopeCannotSmuggleAFireworkIntoProofOnlyHandoff() throws Exception {
        setupFlight();
        var checkpoint = source();
        var firework = new ElytraFlightController.Decision(0, -20, true,
                ElytraFlightController.Phase.CRUISE);
        var handoff = List.of(new FlightControlAdmission.Step(firework, checkpoint.state()));

        assertThrows(IllegalArgumentException.class, () -> new FlightControlAdmission.Envelope(
                owner, checkpoint, steps(0, 0), null, () -> {}, steps(0, -20), handoff, null,
                true));
    }

    @Test void newlyAppearedFlightOneRocketRejectsOrdinaryPrefixPublishedWithEmptyHotbar() throws Exception {
        World roof = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return position.y < 128
                        ? new BlockState("bedrock", 1, Map.of(), "block", -1, true, "STONE")
                        : super.getBlockStateAt(position);
            }
        };
        var worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, roof);
        movementSync.position.set(new Vector3d(1000.53, 129.08, .5));
        movementSync.velocity.set(new Vector3d(.018, -.092, 0));
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        var checkpoint = FlightControlAdmission.snapshot(movementSync);
        var handoff = new LocalClimbHandoff();
        handoff.deployed(129.29);
        var route = handoff.route(checkpoint.state(), new Vector3d(1600, 128, 0), -90);
        var callback = new AtomicInteger();
        assertArrayEquals(new int[0], movementSync.inventoryManager.getHotbarFireworkFlightDurations());

        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, checkpoint, route, route.goal(), true,
                    callback::incrementAndGet, null,
                    new HazardRecovery.Context(new int[] {1}, 0, false, null), true));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertNotNull(controller.latest());
            assertTrue(controller.latest().rocket(), "fixture must select the ordinary powered local-climb prefix");
            var controlsField = MovementSync.class.getDeclaredField("flightControls");
            controlsField.setAccessible(true);
            var pendingField = FlightControlAdmission.class.getDeclaredField("pending");
            pendingField.setAccessible(true);
            var envelope = (FlightControlAdmission.Envelope) pendingField.get(controlsField.get(movementSync));
            assertNotNull(envelope, "ordinary publication must accept the proved rejected-request branch");
            assertEquals(FireworkSchedule.lifetimeBounds(1), envelope.rocketLifetime());
            assertTrue(envelope.steps().get(0).control().firework());
            assertArrayEquals(new int[0], movementSync.inventoryManager.getHotbarFireworkFlightDurations(),
                    "actual hotbar must remain empty through publication despite the provided Flight1 context");
            assertEquals(0, callback.get(), "publication cannot dispatch the action");

            movementSync.inventoryManager.setSlot(0, 36, item("firework_rocket", 1));
            assertArrayEquals(new int[] {1}, movementSync.inventoryManager.getHotbarFireworkFlightDurations());
            assertTrue(movementSync.fireworkRequestReady(), "only inventory changed, not the request latch");
            assertFalse(movementSync.hasAttachedFirework());

            movementSync.snapshotInput();

            assertEquals(0, callback.get(),
                    "an empty-publication capability cannot authorize a newly appeared rocket, even Flight1");
            assertTrue(movementSync.flightControlOwnership().recovering(),
                    "rejecting the request must retain the independently revalidated no-thrust reserve");
            assertNull(movementSync.pollBeforeMovementPacket(), "no rocket action may be queued");
        }
    }

    @Test void boundedLifetimesAdmitFromLiveStateAndDispatchExactlyOnce() throws Exception {
        setupFlight();
        for (int flight = 0; flight <= 1; flight++) {
            admission.clear();
            admission.claim(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
            var checkpoint = source();
            var callback = new AtomicInteger();
            var envelope = lifetimeEnvelope(checkpoint, flight, wideForwardRoute(checkpoint), callback);
            movementSync.position.set(checkpoint.state().position().add(1, 0, 0));
            movementSync.completePhysicsStep();
            admission.submit(envelope);
            admission.consume(movementSync);
            assertEquals(1, callback.get(), "admission must rebase every legal lifetime to live state");
            assertTrue(admission.ownershipTrace().active());
            admission.consume(movementSync);
            admission.submit(envelope);
            admission.consume(movementSync);
            assertEquals(1, callback.get(), "repeated consumption/publication cannot repeat the request");
        }
    }

    @Test void boundedLifetimeCannotAdmitFromLiveStateOutsidePoweredCorridor() throws Exception {
        setupFlight();
        var checkpoint = source();
        var callback = new AtomicInteger();
        var goal = checkpoint.state().position().add(0, 0, 100);
        var corridor = new WaypointFlightTarget(List.of(checkpoint.state().position(), goal),
                goal, false, false, 20);
        var envelope = lifetimeEnvelope(checkpoint, 1, corridor, callback);
        movementSync.position.set(checkpoint.state().position().add(40, 0, 0));
        movementSync.completePhysicsStep();
        admission.submit(envelope);
        admission.consume(movementSync);
        assertEquals(0, callback.get());
        assertFalse(admission.ownershipTrace().active());
    }

    @Test void requestRelativeLifetimeSurvivesDelayedAttachmentAndReserveHandoff() throws Exception {
        setupFlight();
        var checkpoint = source();
        var callback = new AtomicInteger();
        var envelope = lifetimeEnvelope(checkpoint, 1, wideForwardRoute(checkpoint), callback);
        admission.submit(envelope);
        admission.consume(movementSync);
        assertEquals(1, callback.get());
        var rocket = new xin.bbtt.Entity.Entity(9001, new java.util.UUID(0, 9001),
                org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType.FIREWORK_ROCKET,
                0, 100, 0, 0, 0, 0, new Vector3d());
        rocket.getMetadata().put("attached_to_target", java.util.OptionalInt.of(movementSync.entityId));
        for (int tick = 0; tick < 32; tick++) {
            if (tick == 2) movementSync.getWorld().getEntities().put(9001, rocket);
            applyCurrentControl();
            // Acknowledgement delay does not extend the physical request-relative lifetime.
            if (tick == 31) movementSync.getWorld().getEntities().remove(9001);
            admission.consume(movementSync);
            assertTrue(admission.ownershipTrace().active(), "lost lifetime owner at tick=" + (tick + 1));
            assertEquals(tick + 1 >= 27, admission.ownershipTrace().recovering());
            assertEquals(1, callback.get());
        }
        movementSync.getWorld().getEntities().remove(9001);
    }

    @Test void rejectedRequestMayTerminateOnSafeSupportOutsidePoweredCorridor() throws Exception {
        setupFlight();
        installLandingWorld("minecraft:stone");
        var initial = new PhysicsState(new Vector3d(.5, 1.45, .5), new Vector3d(0, -.1, .3),
                false, 0, PlayerPose.GLIDING);
        movementSync.position.set(initial.position());
        movementSync.velocity.set(initial.velocity());
        var checkpoint = source();
        var callback = new AtomicInteger();
        var corridor = poweredLifetimeCorridor(checkpoint.state(), 1);
        var envelope = lifetimeEnvelope(checkpoint, 1, corridor, callback);
        var rejected = checkpoint.state();
        boolean departed = false;
        var executable = new java.util.ArrayList<>(envelope.steps());
        executable.addAll(envelope.recovery());
        for (var step : executable) {
            var control = step.control();
            var result = new GlidingPhysics(control.yaw(), control.pitch(), false, PlayerInput.NONE,
                    movementSync.getMovementParameters()).step(movementSync.getWorld(), rejected,
                            PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            departed |= !corridor.contains(rejected.position(), result.position());
            rejected = FlightSafety.nextState(result);
            if (rejected.onGround()) break;
        }
        assertTrue(departed, "fixture must actually leave the powered corridor");
        assertTrue(rejected.onGround(), "rejection must terminate within executable controls");
        assertTrue(FlightHazards.safeLanding(movementSync.getWorld(), rejected.position()));
        admission.submit(envelope);
        admission.consume(movementSync);
        assertEquals(1, callback.get());
        assertTrue(admission.ownershipTrace().active());
    }

    @Test void immediateOnlyWitnessCannotDispatchBeforeDelayedObservationIsProven() throws Exception {
        setupFlight();
        installLandingWorld("minecraft:stone");
        movementSync.position.set(new Vector3d(.5, 1.45, .5));
        movementSync.velocity.set(new Vector3d(0, -.1, .3));
        var checkpoint = source();
        var callback = new AtomicInteger();
        var envelope = lifetimeEnvelope(checkpoint, 1,
                poweredLifetimeCorridor(checkpoint.state(), 1, 0), callback);

        admission.submit(envelope);
        admission.consume(movementSync);

        assertEquals(0, callback.get(), "prove delayed observation before dispatch, not after it");
        assertFalse(admission.ownershipTrace().active());
    }

    @Test void airborneRejectedRequestCannotEscapeItsCorridorWithoutTerminalSupport() throws Exception {
        setupFlight();
        var checkpoint = source();
        var callback = new AtomicInteger();
        var envelope = lifetimeEnvelope(checkpoint, 1,
                poweredLifetimeCorridor(checkpoint.state(), 1), callback);

        admission.submit(envelope);
        admission.consume(movementSync);

        assertEquals(0, callback.get());
        assertFalse(admission.ownershipTrace().active());
    }

    @Test void rejectedRequestCannotTerminateOnHotSupportOutsidePoweredCorridor() throws Exception {
        setupFlight();
        installLandingWorld("minecraft:magma_block");
        movementSync.position.set(new Vector3d(.5, 1.45, .5));
        movementSync.velocity.set(new Vector3d(0, -.1, .3));
        var checkpoint = source();
        var callback = new AtomicInteger();
        var envelope = lifetimeEnvelope(checkpoint, 1,
                poweredLifetimeCorridor(checkpoint.state(), 1), callback);
        admission.submit(envelope);
        admission.consume(movementSync);
        assertEquals(0, callback.get());
        assertFalse(admission.ownershipTrace().active());
    }

    @Test void rejectionCannotUseProofOnlyHandoffAsItsLandingControl() throws Exception {
        setupFlight();
        installLandingWorld("minecraft:stone");
        movementSync.position.set(new Vector3d(.5, 1.45, .5));
        movementSync.velocity.set(new Vector3d(0, -.1, .3));
        var checkpoint = source();
        var callback = new AtomicInteger();
        var complete = lifetimeEnvelope(checkpoint, 1,
                poweredLifetimeCorridor(checkpoint.state(), 1), callback);
        var controls = new java.util.ArrayList<>(complete.steps());
        controls.addAll(complete.recovery());
        controls.addAll(complete.handoff());
        var envelope = FlightControlAdmission.createPrefix(owner, checkpoint,
                controls.subList(0, 1), List.of(), controls.subList(1, controls.size()),
                complete.corridor(), callback::incrementAndGet, true, complete.rocketLifetime());
        admission.submit(envelope);
        admission.consume(movementSync);
        assertEquals(0, callback.get(), "off-route landing must be executable, not merely predicted");
        assertFalse(admission.ownershipTrace().active());
    }

    private void installLandingWorld(String support) throws Exception {
        var block = new xin.bbtt.Block.BlockState(support, 1, java.util.Map.of(),
                "block", 1.5, true, "STONE");
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public xin.bbtt.Block.BlockState getBlockStateAt(Vector3d position) {
                return position.y < 1 ? block : super.getBlockStateAt(position);
            }
        });
    }

    private WaypointFlightTarget poweredLifetimeCorridor(PhysicsState initial, int flight) {
        return poweredLifetimeCorridor(initial, flight, 1);
    }

    private WaypointFlightTarget poweredLifetimeCorridor(PhysicsState initial, int flight,
            int maximumAcknowledgementDelay) {
        var lifetime = FireworkSchedule.lifetimeBounds(flight);
        var paths = new java.util.ArrayList<List<Vector3d>>();
        for (int poweredTicks = lifetime.minimumPoweredTicks();
                poweredTicks <= lifetime.maximumPoweredTicks(); poweredTicks++) {
            for (int acknowledgementDelay = 0; acknowledgementDelay <= maximumAcknowledgementDelay;
                    acknowledgementDelay++) {
                var path = new java.util.ArrayList<Vector3d>();
                var state = initial;
                path.add(state.position());
                for (int tick = 0; tick < lifetime.maximumPoweredTicks() + 2; tick++) {
                    var result = new GlidingPhysics(0, -20,
                            tick >= acknowledgementDelay && tick < poweredTicks, PlayerInput.NONE,
                            movementSync.getMovementParameters()).step(movementSync.getWorld(), state,
                                    PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
                    assertFalse(result.onGround(), "all accepted powered lifetimes must remain airborne");
                    state = FlightSafety.nextState(result);
                    path.add(state.position());
                }
                paths.add(path);
            }
        }
        return WaypointFlightTarget.executionWitness(paths, paths.get(0).get(paths.get(0).size() - 1), false);
    }

    private FlightControlAdmission.Envelope lifetimeEnvelope(FlightControlAdmission.Source checkpoint,
            int flight, WaypointFlightTarget corridor, AtomicInteger callback) {
        var lifetime = FireworkSchedule.lifetimeBounds(flight);
        int prefixTicks = lifetime.maximumPoweredTicks() + 1 - 6;
        var cruise = new ElytraFlightController.Decision(0, -20, false,
                ElytraFlightController.Phase.CRUISE);
        var prefix = new java.util.ArrayList<FlightControlAdmission.Step>();
        prefix.add(new FlightControlAdmission.Step(new ElytraFlightController.Decision(0, -20, true,
                ElytraFlightController.Phase.CRUISE), checkpoint.state()));
        for (int tick = 1; tick < prefixTicks; tick++)
            prefix.add(new FlightControlAdmission.Step(cruise, checkpoint.state()));
        var reserve = java.util.Collections.nCopies(6,
                new FlightControlAdmission.Step(cruise, checkpoint.state()));
        var handoff = List.of(new FlightControlAdmission.Step(cruise, checkpoint.state()));
        return FlightControlAdmission.createPrefix(owner, checkpoint, prefix, reserve, handoff,
                corridor, callback::incrementAndGet, true, lifetime);
    }

    private void applyCurrentControl() {
        var state = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(),
                movementSync.onGround.get(), movementSync.fallDistance.get(), PlayerPose.GLIDING);
        var result = new GlidingPhysics(movementSync.yaw.get(), movementSync.pitch.get(),
                movementSync.hasAttachedFirework(), PlayerInput.NONE,
                movementSync.getMovementParameters()).step(movementSync.getWorld(), state,
                        PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
        movementSync.position.set(result.position());
        movementSync.velocity.set(result.velocity());
        movementSync.onGround.set(result.onGround());
        movementSync.fallDistance.set(result.fallDistance());
        movementSync.completePhysicsStep();
    }

    @Test void movingPursuitCannotAdmitRebasedGroundContact() throws Exception {
        setupFlight();
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) { return position.y < 1 ? 1 : 0; }
        });
        var planned = new PhysicsState(new Vector3d(.5, 2, .5), new Vector3d(0, -.2, .25), false,
                0, PlayerPose.GLIDING);
        var source = new FlightControlAdmission.Source(movementSync.physicsTickVersion(),
                movementSync.getNavigationGeneration(), planned, admission.epoch());
        var control = new ElytraFlightController.Decision(0, 85, false,
                ElytraFlightController.Phase.CRUISE);
        var coast = new ElytraFlightController.Decision(0, 0, false,
                ElytraFlightController.Phase.CRUISE);
        var pursuit = new WaypointFlightTarget(List.of(planned.position(), new Vector3d(.5, 2, 20)),
                new Vector3d(.5, 2, 20), false, false, 10).asPursuit();
        var envelope = FlightControlAdmission.createPrefix(owner, source,
                List.of(new FlightControlAdmission.Step(control, planned)),
                List.of(new FlightControlAdmission.Step(coast, planned)), pursuit, () -> {}, true);
        var live = new PhysicsState(new Vector3d(.5, 1.02, .5), new Vector3d(0, -.2, .25), false,
                0, PlayerPose.GLIDING);
        assertTrue(new GlidingPhysics(coast.yaw(), coast.pitch(), false, PlayerInput.NONE,
                movementSync.getMovementParameters()).step(movementSync.getWorld(), live,
                        PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height()).onGround(),
                "fixture reserve also touches down when rebased");
        movementSync.position.set(live.position());
        movementSync.velocity.set(live.velocity());
        movementSync.onGround.set(false);

        admission.submit(envelope);
        admission.consume(movementSync);

        assertFalse(admission.ownershipTrace().active(),
                "moving-target pursuit must reject a rebased control that now touches down");
        assertNotNull(admission.failure());
    }
}
