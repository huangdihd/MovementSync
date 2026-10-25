package xin.bbtt.physics;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
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
