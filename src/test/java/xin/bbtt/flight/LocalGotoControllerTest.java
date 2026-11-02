package xin.bbtt.flight;

import java.util.List;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

class LocalGotoControllerTest extends ElytraCommandTestSupport {
    private static final Vector3d GOAL = new Vector3d(0, 100, 40);
    private static World air() {
        return new World() { @Override public boolean chunkLoaded(int x, int z) { return true; } };
    }
    private WaypointFlightTarget route() {
        return new WaypointFlightTarget(List.of(new Vector3d(0, 100, 0), GOAL), GOAL, true);
    }
    private FlightControlAdmission.Source source(World world, Object owner) throws Exception {
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, world);
        movementSync.position.set(new Vector3d(0, 100, 0));
        movementSync.velocity.set(new Vector3d(0, 0, .3));
        movementSync.onGround.set(false);
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        return FlightControlAdmission.snapshot(movementSync);
    }

    @Test void workerIsOffThreadDropsBacklogAndRejectsEpochStalePublication() throws Exception {
        var entered = new CountDownLatch(1);
        var release = new CountDownLatch(1);
        var queries = new AtomicInteger();
        Thread caller = Thread.currentThread();
        World gated = new World() {
            @Override public boolean chunkLoaded(int x, int z) {
                assertNotSame(caller, Thread.currentThread());
                queries.incrementAndGet();
                entered.countDown();
                try { assertTrue(release.await(5, TimeUnit.SECONDS)); }
                catch (InterruptedException error) { throw new AssertionError(error); }
                return true;
            }
        };
        Object owner = new Object();
        var source = source(gated, owner);
        var accepted = new AtomicInteger();
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route(), GOAL, true, accepted::incrementAndGet));
            assertTrue(entered.await(5, TimeUnit.SECONDS));
            try {
                assertFalse(controller.request(movementSync, owner, source, route(), GOAL));
                movementSync.invalidateFlightControls();
            } finally { release.countDown(); }
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertNull(controller.latest());
            assertEquals(0, accepted.get());
            assertTrue(queries.get() > 0);
        }
    }

    @Test void sameTickNewerCheckpointRejectsBlockedOlderWorkerPublication() throws Exception {
        var entered = new CountDownLatch(1);
        var release = new CountDownLatch(1);
        World gated = new World() {
            @Override public boolean chunkLoaded(int x, int z) {
                entered.countDown();
                try { assertTrue(release.await(5, TimeUnit.SECONDS)); }
                catch (InterruptedException error) { throw new AssertionError(error); }
                return true;
            }
        };
        Object owner = new Object();
        var older = source(gated, owner);
        var olderRoute = route();
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, older, olderRoute, GOAL));
            assertTrue(entered.await(5, TimeUnit.SECONDS));
            movementSync.position.set(new Vector3d(1, 100, 0));
            var newer = FlightControlAdmission.snapshot(movementSync);
            assertEquals(older.tick(), newer.tick());
            var newerGoal = new Vector3d(40, 100, 0);
            var newerRoute = new WaypointFlightTarget(
                    List.of(newer.state().position(), newerGoal), newerGoal, false, false, 64);
            assertTrue(movementSync.registerForwardFlightEmergency(owner, newer, -90, newerRoute));
            release.countDown();
            controller.awaitIdle(5, TimeUnit.SECONDS);

            assertNull(controller.latest(),
                    "the older worker must not publish beside the newer same-tick checkpoint");
            var controls = MovementSync.class.getDeclaredField("flightControls");
            controls.setAccessible(true);
            var pending = FlightControlAdmission.class.getDeclaredField("pending");
            pending.setAccessible(true);
            assertNull(pending.get(controls.get(movementSync)));
        } finally {
            release.countDown();
        }
    }

    @Test void busyWorkerObservesReplacementRouteAndCannotPublishObsoleteGeometry() throws Exception {
        var entered = new CountDownLatch(1);
        var release = new CountDownLatch(1);
        World gated = new World() {
            @Override public boolean chunkLoaded(int x, int z) {
                entered.countDown();
                try { assertTrue(release.await(5, TimeUnit.SECONDS)); }
                catch (InterruptedException error) { throw new AssertionError(error); }
                return true;
            }
        };
        Object owner = new Object();
        var source = source(gated, owner);
        var replacementGoal = new Vector3d(40, 100, 0);
        var replacementRoute = new WaypointFlightTarget(
                List.of(source.state().position(), replacementGoal), replacementGoal, false, false, 64);
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route(), GOAL));
            assertTrue(entered.await(5, TimeUnit.SECONDS));
            try {
                assertFalse(controller.request(movementSync, owner, source,
                        replacementRoute, replacementGoal), "single-flight must not queue a second worker");
            } finally {
                release.countDown();
            }
            controller.awaitIdle(5, TimeUnit.SECONDS);

            assertNull(controller.latest(), "route replacement must revoke the old worker's publication rights");
            assertTrue(controller.request(movementSync, owner, source, replacementRoute, replacementGoal),
                    "the latest route must remain eligible after the obsolete worker exits");
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertEquals(LocalGotoController.Status.SAFE_PREFIX, controller.latest().status());
            assertTrue(pendingEnvelope().corridor().sameCorridor(replacementRoute));
        } finally {
            release.countDown();
        }
    }

    @Test void invalidatedRouteCannotPublishOrRemainPendingAndReplacementSteersForward() throws Exception {
        var entered = new CountDownLatch(1);
        var release = new CountDownLatch(1);
        World gated = new World() {
            @Override public boolean chunkLoaded(int x, int z) {
                entered.countDown();
                try { assertTrue(release.await(5, TimeUnit.SECONDS)); }
                catch (InterruptedException error) { throw new AssertionError(error); }
                return true;
            }
        };
        Object owner = new Object();
        var source = source(gated, owner);
        var replacementGoal = new Vector3d(40, 100, 0);
        var replacementRoute = new WaypointFlightTarget(
                List.of(source.state().position(), replacementGoal), replacementGoal, false, false, 64);
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route(), GOAL));
            assertTrue(entered.await(5, TimeUnit.SECONDS));
            try {
                controller.invalidateRoute(movementSync, owner);
            } finally {
                release.countDown();
            }
            controller.awaitIdle(5, TimeUnit.SECONDS);

            assertNull(controller.latest(), "an invalidated worker cannot publish its obsolete route");
            assertNull(pendingEnvelopeValue(), "invalidation must also remove an unpublished old envelope");
            assertTrue(controller.request(movementSync, owner, source, replacementRoute, replacementGoal));
            controller.awaitIdle(5, TimeUnit.SECONDS);

            var envelope = pendingEnvelope();
            assertTrue(envelope.corridor().sameCorridor(replacementRoute));
            float yaw = envelope.steps().get(0).control().yaw();
            var tangent = new Vector3d(replacementGoal).sub(source.state().position()).normalize();
            double headingDot = -Math.sin(Math.toRadians(yaw)) * tangent.x
                    + Math.cos(Math.toRadians(yaw)) * tangent.z;
            assertTrue(headingDot > 0, "replacement control must point into its forward tangent");
        } finally {
            release.countDown();
        }
    }

    @Test void unknownTerrainIsExplicitUnprovenNotLandingOrRocket() throws Exception {
        Object owner = new Object();
        var source = source(new World() {}, owner);
        var accepted = new AtomicInteger();
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route(), GOAL, true, accepted::incrementAndGet));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertEquals(LocalGotoController.Status.UNPROVEN, controller.latest().status());
            assertEquals(0, controller.latest().prefixTicks());
            assertEquals(0, controller.latest().reserveTicks());
            assertFalse(controller.latest().rocket());
            assertEquals(0, accepted.get());
        }
    }

    @Test void openAirProducesFinitePrefixWithoutExecutingCallbackOnWorker() throws Exception {
        Object owner = new Object();
        var source = source(air(), owner);
        var accepted = new AtomicInteger();
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route(), GOAL, accepted::incrementAndGet));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            var result = controller.latest();
            assertEquals(LocalGotoController.Status.SAFE_PREFIX, result.status());
            assertTrue(result.prefixTicks() > 0 && result.prefixTicks() <= 12);
            assertTrue(result.reserveTicks() >= 0 && result.reserveTicks() <= 6);
            assertFalse(result.rocket());
            assertEquals(0, accepted.get());
            assertEquals(source.tick(), result.tick());
            assertEquals(source.epoch(), result.epoch());
            assertEquals(source.generation(), result.generation());
            assertFalse(controller.request(movementSync, owner, source, route(), GOAL));
        }
    }

    @Test void movingTargetPublishesAirborneOnlyAdmissionEnvelope() throws Exception {
        World world = air();
        Object owner = new Object();
        var source = source(world, owner);
        var route = LowAltitudePlanner.plan(world, source.state(), GOAL, MovementParameters.DEFAULT);
        assertNotNull(route);
        assertTrue(route.terrainValidated());
        var tracking = new LocalLookaheadSolver.MovingTarget(new Vector3d(GOAL), new Vector3d());
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route, GOAL, false, () -> {}, tracking));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertEquals(LocalGotoController.Status.SAFE_PREFIX, controller.latest().status());
            var envelope = pendingEnvelope();
            assertTrue(envelope.requireAirborne());
            assertNotNull(envelope.corridor(),
                    "a live endpoint may trigger replanning but cannot remove the validated route corridor");
            assertTrue(envelope.corridor().sameCorridor(route));
        }
    }

    @Test void publishedPrefixRetainsRouteForLiveCorridorRevalidation() throws Exception {
        World world = air();
        Object owner = new Object();
        var source = source(world, owner);
        var route = LowAltitudePlanner.plan(world, source.state(), GOAL, MovementParameters.DEFAULT);
        assertNotNull(route);
        assertTrue(route.terrainValidated());
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route, GOAL));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertEquals(LocalGotoController.Status.SAFE_PREFIX, controller.latest().status());
            var corridor = assertInstanceOf(WaypointFlightTarget.class, pendingEnvelope().corridor());
            assertTrue(corridor.terrainValidated(),
                    "admission must retain the planner's live swept-volume validation");
            assertEquals(route.waypoints(), corridor.waypoints());
            assertEquals(route.corridorRadius(), corridor.corridorRadius());
        }
    }

    @Test void staticNonPlannerRouteStillRetainsItsGeometricCorridor() throws Exception {
        World world = air();
        Object owner = new Object();
        var source = source(world, owner);
        var route = new WaypointFlightTarget(
                List.of(source.state().position(), GOAL), GOAL, false, false, 6);
        assertFalse(route.terrainValidated());
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route, GOAL));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertEquals(LocalGotoController.Status.SAFE_PREFIX, controller.latest().status());
            var corridor = assertInstanceOf(WaypointFlightTarget.class, pendingEnvelope().corridor());
            assertEquals(route.waypoints(), corridor.waypoints());
            assertEquals(route.corridorRadius(), corridor.corridorRadius());
        }
    }

    private FlightControlAdmission.Envelope pendingEnvelope() throws Exception {
        return assertInstanceOf(FlightControlAdmission.Envelope.class, pendingEnvelopeValue());
    }

    private Object pendingEnvelopeValue() throws Exception {
        var controls = MovementSync.class.getDeclaredField("flightControls");
        controls.setAccessible(true);
        var pending = FlightControlAdmission.class.getDeclaredField("pending");
        pending.setAccessible(true);
        return pending.get(controls.get(movementSync));
    }

    @Test void equalGeometryRenewalCannotReuseWorkerCursorBehindLiveRouteIndex() throws Exception {
        World world = air();
        Object owner = new Object();
        var initial = source(world, owner);
        var a = initial.state().position();
        var b = new Vector3d(0, 100, 10);
        var c = new Vector3d(10, 100, 10);
        var target = new WaypointFlightTarget(List.of(a, b, c), c, false, false, 6);
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, initial, target, c));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertEquals(0, controller.latest().routeCursor(), 1.0e-6);

            assertTrue(target.reconcile(world, new PhysicsState(new Vector3d(1, 100, 10.1),
                    new Vector3d(1, 0, 0), false)));
            assertEquals(2, target.index());
            movementSync.completePhysicsStep();
            movementSync.position.set(new Vector3d(0, 100, 5));
            movementSync.velocity.set(new Vector3d(0, 0, .3));
            var corrected = FlightControlAdmission.snapshot(movementSync);

            assertTrue(controller.request(movementSync, owner, corrected, target, c));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertTrue(controller.latest().routeCursor() >= a.distance(b),
                    "worker steering must not reintroduce a segment consumed by the live route cursor");
        }
    }

    @Test void tickZeroDeduplicatesStateButAllowsLegacyDriverToAdvance() throws Exception {
        Object owner = new Object();
        var initial = source(air(), owner);
        assertEquals(0, initial.tick());
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, initial, route(), GOAL));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertFalse(controller.request(movementSync, owner, initial, route(), GOAL));
            movementSync.position.set(new Vector3d(0, 100, 1));
            var advanced = FlightControlAdmission.snapshot(movementSync);
            assertEquals(initial.tick(), advanced.tick());
            assertTrue(controller.request(movementSync, owner, advanced, route(), GOAL));
            controller.awaitIdle(5, TimeUnit.SECONDS);
        }
    }

    @Test void nearRoofLocalClimbDispatchesItsProvenInitialRocketBeforeGroundContact() throws Exception {
        World roof = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                return p.y < 128 ? new BlockState("bedrock", 1, Map.of(), "block", -1, true, "STONE")
                        : super.getBlockStateAt(p);
            }
        };
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, roof);
        movementSync.position.set(new Vector3d(1000.53, 129.08, .5));
        movementSync.velocity.set(new Vector3d(.018, -.092, 0));
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        Object owner = new Object();
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        var source = FlightControlAdmission.snapshot(movementSync);
        var handoff = new LocalClimbHandoff();
        handoff.deployed(129.29);
        var route = handoff.route(source.state(), new Vector3d(1600, 128, 0), -90);
        var accepted = new AtomicInteger();
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route, route.goal(), true,
                    accepted::incrementAndGet, null,
                    new HazardRecovery.Context(new int[] {1}, 0, false, null), true));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertTrue(controller.latest().rocket(), "fixture must choose the powered local-climb prefix");
            var envelope = pendingEnvelope();
            assertTrue(envelope.steps().size() >= 6, "fixture must prove a six-tick powered prefix");
            var state = source.state();
            for (int i = 0; i < 6; i++) {
                var control = envelope.steps().get(i).control();
                var result = new GlidingPhysics(control.yaw(), control.pitch(), true, PlayerInput.NONE,
                        MovementParameters.DEFAULT).step(roof, state, PlayerPose.HALF_WIDTH,
                                PlayerPose.GLIDING.height());
                assertFalse(result.onGround(), "powered climb touched down at replay step " + i);
                assertTrue(envelope.corridor().contains(state.position(), result.position()),
                        "physical powered climb left its bounded route corridor at replay step " + i);
                state = new PhysicsState(result.position(), result.velocity(), result.onGround(),
                        result.fallDistance(), result.pose());
            }

            movementSync.snapshotInput();

            assertEquals(1, accepted.get(),
                    "the admitted near-roof climb must dispatch its proven initial rocket");
        }
    }

    @Test void ridgePursuitPreparesTurningEpisodeBeforeComparingConstantControlHorizons() throws Exception {
        World ridge = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return position.y < 64 || position.x >= 112 && position.x < 128 && position.y < 84
                        ? new BlockState("stone", 1, Map.of(), "block", 1.5, true, "STONE")
                        : super.getBlockStateAt(position);
            }
        };
        Object owner = new Object();
        source(ridge, owner);
        movementSync.position.set(new Vector3d(110.9, 65.95, 10.98));
        movementSync.velocity.set(new Vector3d(-.01365, -.1822, .08598));
        movementSync.fallDistance.set(1.1822);
        movementSync.setGliding(true);
        movementSync.yaw.set(-90f);
        movementSync.pitch.set(-5f);
        inventory[36] = item("firework_rocket", 1);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        var source = FlightControlAdmission.snapshot(movementSync);
        var goal = new Vector3d(143.7, 64, 8.5);
        var routeSource = new PhysicsState(new Vector3d(111, 66.7, 10.31),
                new Vector3d(), false, 1, PlayerPose.GLIDING);
        var route = LowAltitudePlanner.planPursuit(ridge, routeSource,
                new Vector3d(157.3, 64, 8.5), MovementParameters.DEFAULT);
        assertNotNull(route);
        var accepted = new AtomicInteger();
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route, goal, true,
                    accepted::incrementAndGet,
                    new LocalLookaheadSolver.MovingTarget(goal, new Vector3d(.8, 0, 0))));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertTrue(controller.latest().powered().prefix().size()
                    < controller.latest().unpowered().prefix().size(),
                    "fixture must exercise the shorter constant-control powered seed");
            assertTrue(controller.latest().rocket(),
                    "a shorter constant-control seed must not veto a fully proved turning climb");
            var envelope = pendingEnvelope();
            assertEquals(27, envelope.steps().size());
            assertEquals(6, envelope.recovery().size());
            assertEquals(1, envelope.handoff().size());
            movementSync.snapshotInput();
            assertEquals(1, accepted.get(), "live admission must accept the complete ridge episode");
        }
    }

    @Test void arrivalDoesNotRequestRocketAtCorrectGoalElevation() throws Exception {
        Object owner = new Object();
        var initial = source(air(), owner);
        Vector3d nearby = new Vector3d(0, 100, 10);
        var target = new WaypointFlightTarget(List.of(initial.state().position(), nearby), nearby, true);
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, initial, target, nearby, true, () -> {}));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertFalse(controller.latest().rocket());
            assertNull(controller.latest().powered(), "arrival must not evaluate unnecessary thrust");
        }
    }

    @Test void lowEnergySupportedApproachCanPrepareProvenRescueBeforePrematureTouchdown() throws Exception {
        World roofed = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return position.y < 64 || position.y >= 80
                        ? new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE")
                        : super.getBlockStateAt(position);
            }
        };
        Object owner = new Object();
        source(roofed, owner);
        movementSync.position.set(new Vector3d(16.5, 65.77, 578.2));
        movementSync.velocity.set(new Vector3d(0, -.0145339, .4171));
        movementSync.fallDistance.set(1.0145339);
        movementSync.setGliding(true);
        inventory[36] = item("firework_rocket", 64);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        var initial = FlightControlAdmission.snapshot(movementSync);
        var goal = new Vector3d(16.5, 64, 608.5);
        assertTrue(LocalGotoController.arrival(roofed, MovementParameters.DEFAULT,
                initial.state(), goal, false, 0).isEmpty());
        var route = LowAltitudePlanner.plan(roofed, initial.state(), goal, MovementParameters.DEFAULT);
        assertNotNull(route);
        var accepted = new AtomicInteger();
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, initial, route, goal, true,
                    accepted::incrementAndGet, null,
                    new HazardRecovery.Context(new int[] {1}, 0, false, null), true));
            assertTrue(controller.awaitIdle(5, TimeUnit.SECONDS));
            assertNotNull(controller.latest().powered(), "proximity cannot disable a needed rescue search");
            assertTrue(controller.latest().rocket(), "rescue must own a complete all-outcome episode");
            movementSync.snapshotInput();
            assertEquals(1, accepted.get());
            assertTrue(Math.cos(Math.toRadians(movementSync.yaw.get())) > 0,
                    "new thrust must point toward the landing destination");
        }
    }

    @Test void partialCruiseBoostsBeforeItsShortLookaheadBecomesAnArtificialEndpoint() throws Exception {
        World roofed = new World() {
            @Override public boolean chunkLoaded(int x, int z) {
                return x >= -2 && x <= 2 && Math.abs(z - 17) <= 3;
            }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return position.y < 64 || position.y >= 80
                        ? new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE")
                        : super.getBlockStateAt(position);
            }
        };
        Object owner = new Object();
        source(roofed, owner);
        movementSync.position.set(new Vector3d(16.5, 65.56, 282.8));
        movementSync.velocity.set(new Vector3d(0, .007474, .7228));
        movementSync.fallDistance.set(1.0);
        movementSync.pitch.set(-20f);
        movementSync.setGliding(true);
        inventory[36] = item("firework_rocket", 64);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        var initial = FlightControlAdmission.snapshot(movementSync);
        var goal = new Vector3d(16.5, 64, 608.5);
        var route = LowAltitudePlanner.plan(roofed, initial.state(), goal, MovementParameters.DEFAULT);
        assertNotNull(route);
        assertTrue(route.partial());
        var accepted = new AtomicInteger();
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, initial, route, goal, true,
                    accepted::incrementAndGet, null,
                    new HazardRecovery.Context(new int[] {1}, 0, false, null), true));
            assertTrue(controller.awaitIdle(5, TimeUnit.SECONDS));
            assertEquals(12, controller.latest().unpowered().prefix().size());
            assertTrue(controller.latest().powered().score() > controller.latest().unpowered().score());
            assertTrue(controller.latest().rocket(), "cruise must not wait for imminent touchdown");
            assertEquals(27, pendingEnvelope().steps().size());
            assertEquals(6, pendingEnvelope().recovery().size());
            assertEquals(1, pendingEnvelope().handoff().size());
            movementSync.snapshotInput();
            assertEquals(1, accepted.get(), "the complete episode must also pass live admission");
        }
    }

    @Test void attachedThrustDoesNotReverseIntoPrematureArrivalBraking() {
        World corridor = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                boolean inside = position.x >= 0 && position.x < 32
                        && position.z >= 0 && position.z < 112
                        || position.x >= 0 && position.x < 112
                        && position.z >= 80 && position.z < 112;
                return position.y < 64 || position.y >= 80 || !inside
                        ? new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE")
                        : super.getBlockStateAt(position);
            }
        };
        var state = new PhysicsState(
                new Vector3d(69.08231906540118, 65.69606990048706, 90.47330942398973),
                new Vector3d(1.6552545383010044, -0.028156689511755997, 0.23422454816256819),
                false, 1, PlayerPose.GLIDING);
        var goal = new Vector3d(100.5, 64, 96.5);
        float staleRouteHeading = 98.10954f;
        float goalHeading = (float) Math.toDegrees(Math.atan2(
                -(goal.x - state.position().x), goal.z - state.position().z));

        var arrival = LocalGotoController.arrival(corridor, MovementParameters.DEFAULT,
                state, goal, true, staleRouteHeading);

        assertTrue(arrival.isEmpty() || Math.cos(Math.toRadians(
                        arrival.get(0).control().yaw() - goalHeading)) > 0,
                "a passed route anchor must not turn attached arrival away from its goal: "
                        + (arrival.isEmpty() ? "no arrival" : arrival.get(0).control()));
    }

    private static World floor(boolean hot) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) { return minY < 10; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return position.y < 10 ? new BlockState(hot ? "magma_block" : "stone", 1,
                        Map.of(), "block", 1.5, true, "STONE") : super.getBlockStateAt(position);
            }
        };
    }
    @Test void landingRequiresTargetHeightHorizontalRadiusActualSupportAndNoHazard() {
        var state = new PhysicsState(new Vector3d(0, 10, 0), new Vector3d(), true);
        assertTrue(LocalGotoController.normalLanding(floor(false), state, new Vector3d(0, 10, 0)));
        assertTrue(LocalGotoController.normalLanding(floor(false), state, new Vector3d(0, 11, 0)));
        assertFalse(LocalGotoController.normalLanding(floor(false), state, new Vector3d(0, 11.01, 0)));
        assertFalse(LocalGotoController.normalLanding(floor(false), state, new Vector3d(2.01, 10, 0)));
        assertFalse(LocalGotoController.normalLanding(air(), state, new Vector3d(0, 10, 0)));
        assertFalse(LocalGotoController.normalLanding(floor(true), state, new Vector3d(0, 10, 0)));
        assertFalse(LocalGotoController.normalLanding(new World() {}, state, new Vector3d(0, 10, 0)));
    }

    @Test void brakingReplayIsFiniteAndCannotTreatWrongHeightGroundAsSuccess() {
        var control = new ElytraFlightController.Decision(0, -45, false, ElytraFlightController.Phase.LANDING);
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(0, 0, .3), false);
        var replay = LocalGotoController.replay(air(), MovementParameters.DEFAULT, initial,
                control, false, 6, GOAL, true);
        assertEquals(6, replay.steps().size());
        assertTrue(replay.steps().stream().noneMatch(step -> step.control().firework()));
        var falling = new PhysicsState(new Vector3d(0, 10.02, 0), new Vector3d(0, -.2, 0), false);
        assertTrue(LocalGotoController.replay(floor(false), MovementParameters.DEFAULT, falling,
                control, false, 6, new Vector3d(0, 100, 0), true).steps().isEmpty());
    }

    @Test void unpoweredSupportedContactRemainsAValidTerminalLanding() {
        var control = new ElytraFlightController.Decision(0, 23, false,
                ElytraFlightController.Phase.LANDING);
        var state = new PhysicsState(new Vector3d(0, 10.02, 0), new Vector3d(0, -.2, .1),
                false, 0, PlayerPose.GLIDING);

        var replay = LocalGotoController.replay(floor(false), MovementParameters.DEFAULT, state,
                control, false, 40, new Vector3d(0, 10, 0), true);

        assertFalse(replay.steps().isEmpty());
        assertTrue(replay.end().onGround());
    }

}
