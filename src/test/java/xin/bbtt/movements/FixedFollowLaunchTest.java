package xin.bbtt.movements;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Map;
import java.util.UUID;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.Entity.Entity;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.flight.LocalGotoController;
import xin.bbtt.physics.FixedFlightTarget;
import xin.bbtt.physics.FlightControlAdmission;
import xin.bbtt.physics.GlidingPhysics;
import xin.bbtt.physics.HazardRecovery;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerInput;
import xin.bbtt.physics.PlayerPose;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

/** Fixed follow keeps landing geometry but must own executable controls at first glide. */
final class FixedFollowLaunchTest extends ElytraCommandTestSupport {
    private static final BlockState STONE = new BlockState("minecraft:stone", 1, Map.of(),
            "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState("minecraft:air", 0, Map.of(),
            "empty", 0, false, "AIR");
    private final Vector3d destination = new Vector3d(100.5, 64, .5);
    private final Object owner = new Object();
    private final AtomicInteger accepted = new AtomicInteger();

    private record Fixture(LowAltitudeFlightPlan.Result plan, HazardRecovery.Context resources,
            FlightControlAdmission.Source source) {}

    private World installGroundedWorld() throws Exception {
        movementSync.getMovementController().pause();
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int floor = 64 + (x > 12 && x < 88 ? Math.floorMod(x, 24) / 6 : 0);
                boolean tree = x > 24 && x < 80 && Math.floorMod(x, 24) < 3
                        && position.z >= 3 && position.z < 6 && position.y < floor + 5;
                return position.y < floor || tree ? STONE : AIR;
            }
        };
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
        movementSync.position.set(new Vector3d(.5, 64, .5));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(true);
        movementSync.setGliding(false);
        inventory[36] = item("firework_rocket", 1);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        return world;
    }

    private HazardRecovery.Context resources() {
        return new HazardRecovery.Context(new int[] {1}, 0, false, movementSync.getLandingMemory());
    }

    private PhysicsState groundedState() {
        return new PhysicsState(movementSync.position.get(), movementSync.velocity.get(), true, 0);
    }

    private Fixture prepare() throws Exception {
        var world = installGroundedWorld();
        var origin = groundedState();
        var resources = resources();
        var parameters = movementSync.getMovementParameters();
        var future = LowAltitudeFlightPlan.fixedFollow(world, origin, destination, parameters, false, resources);
        var plan = future.get(30, TimeUnit.SECONDS);
        assertEquals(LowAltitudeFlightPlan.Status.COMPLETE, future.status());
        assertFixedLaunch(plan);
        assertEquals(origin.position(), plan.origin());
        assertTrue(plan.preparedLaunch().matches(resources));

        // Use the exact physical deployment and first unpowered glide, not a synthetic airborne state.
        var deployment = LowAltitudeFlightPlan.deployment(world, origin, parameters, plan.yaw(), plan.pitch());
        assertNotNull(deployment);
        var first = new GlidingPhysics(plan.yaw(), plan.pitch(), false, PlayerInput.NONE, parameters)
                .step(world, deployment, PlayerPose.HALF_WIDTH, PlayerPose.STANDING.height());
        assertFalse(first.onGround());
        movementSync.position.set(first.position());
        movementSync.velocity.set(first.velocity());
        movementSync.fallDistance.set(first.fallDistance());
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        return new Fixture(plan, resources, FlightControlAdmission.snapshot(movementSync));
    }

    private void assertFixedLaunch(LowAltitudeFlightPlan.Result plan) {
        assertNotNull(plan);
        assertEquals(destination, plan.destination());
        assertEquals(destination, plan.route().goal());
        assertFalse(plan.route().pursuit(), "lost follow must retain fixed landing intent");
        assertNotNull(plan.preparedLaunch(), "fixed follow cannot deploy with geometry alone");
    }

    private boolean publish(LocalGotoController controller, Fixture fixture, HazardRecovery.Context resources) {
        return controller.publishLaunch(movementSync, owner, fixture.source(), fixture.plan().preparedLaunch(),
                fixture.plan().route(), destination, resources, accepted::incrementAndGet);
    }

    @Test void fixedFollowAdmitsThirtyThreeExecutableTicksExactlyOnceWithoutWorker() throws Exception {
        var fixture = prepare();
        try (var controller = new LocalGotoController()) {
            assertTrue(publish(controller, fixture, fixture.resources()));
            assertEquals(0, accepted.get(), "publication alone cannot execute launch controls");
            assertNull(controller.latest());

            movementSync.snapshotInput();

            assertEquals(1, accepted.get());
            var ownership = movementSync.flightControlOwnership();
            assertTrue(ownership.active());
            assertEquals(33, ownership.remainingExecutableTicks());
            assertTrue(Float.isFinite(movementSync.yaw.get()));
            assertTrue(Float.isFinite(movementSync.pitch.get()));
            movementSync.snapshotInput();
            assertEquals(1, accepted.get(), "repeated ordinary input must not repeat the launch action");
            assertNull(controller.latest(), "first glide must not depend on a cold worker request");
        }
    }

    @Test void ordinaryStaticSubmissionKeepsGeometryOnlyBehavior() throws Exception {
        var world = installGroundedWorld();
        var plan = LowAltitudeFlightPlan.submit(world, groundedState(), destination,
                movementSync.getMovementParameters(), false, resources()).get(30, TimeUnit.SECONDS);
        assertNotNull(plan);
        assertEquals(destination, plan.destination());
        assertFalse(plan.route().pursuit());
        assertNull(plan.preparedLaunch(), "fixed follow must not change ordinary static travel");
    }

    @Test void alreadyGlidingFixedFollowDoesNotPrepareAnotherLaunch() throws Exception {
        var fixture = prepare();
        var plan = LowAltitudeFlightPlan.fixedFollow(movementSync.getWorld(), fixture.source().state(),
                destination, movementSync.getMovementParameters(), true, fixture.resources())
                .get(30, TimeUnit.SECONDS);
        assertNotNull(plan);
        assertEquals(destination, plan.destination());
        assertFalse(plan.route().pursuit());
        assertNull(plan.preparedLaunch(), "only predeployment planning needs launch controls");
    }

    @Test void coolingOrAttachedResourcesRejectGroundedFixedFollowPreparation() throws Exception {
        var world = installGroundedWorld();
        for (var resources : List.of(
                new HazardRecovery.Context(new int[] {1}, 1, false, movementSync.getLandingMemory()),
                new HazardRecovery.Context(new int[] {1}, 0, true, movementSync.getLandingMemory()))) {
            var future = LowAltitudeFlightPlan.fixedFollow(world, groundedState(), destination,
                    movementSync.getMovementParameters(), false, resources);
            assertNull(future.get(30, TimeUnit.SECONDS));
            assertEquals(LowAltitudeFlightPlan.Status.DEPLOYMENT_REJECTED, future.status());
        }
    }

    @Test void absentCoolingOrAttachedResourcesCannotReuseFixedFollowLaunch() throws Exception {
        var fixture = prepare();
        try (var controller = new LocalGotoController()) {
            for (var resources : List.of(
                    new HazardRecovery.Context(new int[0], 0, false, movementSync.getLandingMemory()),
                    new HazardRecovery.Context(new int[] {1}, 1, false, movementSync.getLandingMemory()),
                    new HazardRecovery.Context(new int[] {1}, 0, true, movementSync.getLandingMemory()))) {
                assertFalse(publish(controller, fixture, resources));
                movementSync.snapshotInput();
                assertEquals(0, accepted.get());
                assertFalse(movementSync.flightControlOwnership().active());
                assertNull(controller.latest());
            }
        }
    }

    @Test void removedLiveRocketRejectsOrdinaryAdmission() throws Exception {
        var fixture = prepare();
        try (var controller = new LocalGotoController()) {
            assertTrue(publish(controller, fixture, fixture.resources()));
            movementSync.inventoryManager.setSlot(0, 36, null);
            movementSync.snapshotInput();
            assertEquals(0, accepted.get());
            assertTrue(movementSync.flightControlOwnership().active());
            assertTrue(movementSync.flightControlOwnership().recovering(),
                    "rocket rejection may retain the independently validated unpowered reserve, not launch controls");
            assertNull(controller.latest());
        }
    }

    @Test void replacementGenerationRejectsPreparedPublication() throws Exception {
        var fixture = prepare();
        movementSync.beginNavigationRequest();
        try (var controller = new LocalGotoController()) {
            assertFalse(publish(controller, fixture, fixture.resources()));
            movementSync.snapshotInput();
            assertEquals(0, accepted.get());
            assertFalse(movementSync.flightControlOwnership().active());
            assertNull(controller.latest());
        }
    }

    @Test void replacementGenerationRevokesPublishedLaunchBeforeAdmission() throws Exception {
        var fixture = prepare();
        try (var controller = new LocalGotoController()) {
            assertTrue(publish(controller, fixture, fixture.resources()));
            movementSync.beginNavigationRequest();
            movementSync.snapshotInput();
            assertEquals(0, accepted.get());
            assertFalse(movementSync.flightControlOwnership().active());
            assertNull(controller.latest());
        }
    }

    @Test void groundedLossHandsPreparedFixedPlanToFollowChild() throws Exception {
        var world = installGroundedWorld();
        var entity = new Entity(99, new UUID(0, 99), EntityType.ARMOR_STAND,
                destination.x, destination.y, destination.z, 0, 0, 0, new Vector3d());
        entity.getMetadata().put("shared_flags", (byte) 0);
        world.getEntities().put(99, entity);
        var follow = new FollowMovement(99, 2, false, movementSync.beginNavigationRequest());
        try {
            follow.init();
            world.getEntities().remove(99);
            follow.onTick();
            var pending = assertInstanceOf(CompletableFuture.class, field(follow, "lowPlan"));
            var plan = assertInstanceOf(LowAltitudeFlightPlan.Result.class, pending.get(30, TimeUnit.SECONDS));
            assertFixedLaunch(plan);

            follow.onTick();

            var child = assertInstanceOf(ElytraFlyMovement.class, field(follow, "child"));
            assertInstanceOf(FixedFlightTarget.class, field(child, "target"));
            assertEquals(destination, child.getTarget());
            assertSame(plan.preparedLaunch(), field(child, "preparedLaunch"));
            assertTrue((boolean) field(follow, "landing"));
            assertFalse(follow.isFinished(), "launch handoff is not arrival");
        } finally {
            follow.onStop();
        }
    }

    private static Object field(Object target, String name) throws Exception {
        Field field = target.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return field.get(target);
    }
}
