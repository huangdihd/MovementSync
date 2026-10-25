package xin.bbtt.flight;

import java.lang.reflect.Field;
import java.util.List;
import java.util.concurrent.TimeUnit;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.ElytraFlightController;
import xin.bbtt.physics.FlightControlAdmission;
import xin.bbtt.physics.LowAltitudePlanner;
import xin.bbtt.physics.MovementParameters;
import xin.bbtt.physics.WaypointFlightTarget;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

class CorridorAdmissionRegressionTest extends ElytraCommandTestSupport {
    private static final Vector3d START = new Vector3d(0, 100, 0);
    private static final Vector3d GOAL = new Vector3d(0, 100, 40);

    private static World air() {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
        };
    }

    private void setup(World world) throws Exception {
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, world);
        movementSync.position.set(START);
        movementSync.velocity.set(new Vector3d(0, 0, .3));
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
    }

    @Test void seventhPrefixTickCannotLeaveCorridor() throws Exception {
        setup(air());
        var admission = new FlightControlAdmission();
        var owner = new Object();
        admission.claim(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        var snapshot = FlightControlAdmission.snapshot(movementSync);
        var source = new FlightControlAdmission.Source(snapshot.tick(), snapshot.generation(),
                snapshot.state(), admission.epoch());
        var control = new ElytraFlightController.Decision(0, 0, false,
                ElytraFlightController.Phase.CRUISE);
        var steps = java.util.stream.IntStream.range(0, 8)
                .mapToObj(ignored -> new FlightControlAdmission.Step(control, source.state()))
                .toList();
        var corridor = new WaypointFlightTarget(List.of(START, GOAL), GOAL, false, false, 10);

        admission.submit(FlightControlAdmission.createPrefix(owner, source, steps, List.of(),
                corridor, () -> {}));
        admission.consume(movementSync);
        assertTrue(admission.ownershipTrace().active());
        for (int tick = 1; tick < 6; tick++) {
            movementSync.completePhysicsStep();
            admission.consume(movementSync);
            assertTrue(admission.ownershipTrace().active(), "prefix stopped before tick " + (tick + 1));
        }

        movementSync.completePhysicsStep();
        movementSync.position.set(new Vector3d(100, 100, 0));
        admission.consume(movementSync);

        assertFalse(admission.ownershipTrace().active(),
                "the seventh prefix tick must be stopped after leaving its corridor");
        assertNotNull(admission.failure());
    }

    @Test void plannerBackedPrefixReserveHandoffRevalidatesSameCorridor() throws Exception {
        World world = air();
        setup(world);
        var owner = new Object();
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        var source = FlightControlAdmission.snapshot(movementSync);
        var route = LowAltitudePlanner.plan(world, source.state(), GOAL, MovementParameters.DEFAULT);
        assertNotNull(route);
        assertTrue(route.terrainValidated());

        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route, GOAL));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertEquals(LocalGotoController.Status.SAFE_PREFIX, controller.latest().status());

            var envelope = pendingEnvelope();
            assertFalse(envelope.recovery().isEmpty(), "fixture must exercise prefix-to-reserve handoff");
            assertNotNull(envelope.corridor(),
                    "a static prefix produced from a planner-backed route must retain its corridor");

            var admission = flightControls();
            movementSync.snapshotInput();
            assertTrue(admission.ownershipTrace().active());
            for (int tick = 1; tick < envelope.steps().size(); tick++) {
                movementSync.completePhysicsStep();
                movementSync.snapshotInput();
                assertTrue(admission.ownershipTrace().active(),
                        "planner prefix stopped before its handoff at tick " + tick);
            }

            movementSync.completePhysicsStep();
            movementSync.position.set(new Vector3d(100, 100, 0));
            movementSync.snapshotInput();

            assertFalse(admission.ownershipTrace().active(),
                    "reserve must inherit and revalidate the planner corridor at handoff");
            assertNotNull(movementSync.getFlightControlFailure());
        }
    }

    private FlightControlAdmission.Envelope pendingEnvelope() throws Exception {
        Field pending = FlightControlAdmission.class.getDeclaredField("pending");
        pending.setAccessible(true);
        return assertInstanceOf(FlightControlAdmission.Envelope.class,
                pending.get(flightControls()));
    }

    private FlightControlAdmission flightControls() throws Exception {
        Field controls = MovementSync.class.getDeclaredField("flightControls");
        controls.setAccessible(true);
        return assertInstanceOf(FlightControlAdmission.class, controls.get(movementSync));
    }
}
