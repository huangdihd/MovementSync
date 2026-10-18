package xin.bbtt.movements;

import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.flight.FlightContextBuilder;
import xin.bbtt.movement.Movement;
import xin.bbtt.pathfinding.BuiltinMovementType;
import xin.bbtt.pathfinding.Node;
import xin.bbtt.pathfinding.PathStep;
import xin.bbtt.world.World;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

final class TravelMovementArrivalRegressionTest {
    private static final Vector3i GOAL = new Vector3i(200, 73, 0);

    private static final class Leg extends Movement {
        @Override public void init() {}
        @Override public void onTick() {}
        @Override public long getTime() { return -1; }
        @Override public void onStop() { finished = true; }
    }

    private static final class Fixture implements TravelMovement.Legs {
        final World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double x, double y, double z,
                                                    double maxX, double maxY, double maxZ) { return false; }
        };
        boolean equipped;
        final MovementSync sync = new MovementSync() {
            @Override public World getWorld() { return world; }
            @Override public boolean isGliding() { return false; }
            @Override public boolean hasEquippedElytra() { return equipped; }
        };
        final long generation = sync.beginStaticNavigationRequest(GOAL);
        final Leg flight = new Leg();
        final Leg walk = new Leg();
        int walks;
        final TravelMovement travel = new TravelMovement(GOAL, generation, true, this);
        Fixture() {
            sync.position.set(new Vector3d(.5, 64, .5));
            sync.onGround.set(true);
        }
        @Override public List<PathStep> plan(Vector3i destination) {
            assertEquals(GOAL, destination, "the remainder must retain the platform elevation");
            return List.of(new PathStep(new Node(200, 64, 0), BuiltinMovementType.WALK),
                    new PathStep(new Node(200, 73, 0), BuiltinMovementType.WALK));
        }
        @Override public Movement walk(List<PathStep> path, Vector3i destination, boolean intermediate) {
            assertFalse(intermediate);
            walks++;
            return walk;
        }
        @Override public Movement flight() { return flight; }
        void landBelowPlatform() {
            travel.onTick();
            sync.position.set(new Vector3d(200.5, 64, .5));
            flight.setFinished(true);
            travel.onTick();
            assertEquals(1, walks);
            assertFalse(travel.isFinished());
            assertTrue(sync.isNavigationRequestCurrent(generation));
        }
    }

    @Test void landingNineBlocksBelowGoalWalksRemainderAndCannotReportArrival() {
        Fixture f = new Fixture();
        f.landBelowPlatform();
        f.walk.setFinished(true);
        f.travel.onTick();
        assertTrue(f.travel.isFinished());
        assertNull(f.sync.getActiveGoal(), "failed walking remainder abandons static navigation");
    }

    @Test void supportedThreeDimensionalArrivalCompletesRemainder() {
        Fixture f = new Fixture();
        f.landBelowPlatform();
        f.sync.position.set(new Vector3d(200.5, 73, .5));
        f.walk.setFinished(true);
        f.travel.onTick();
        assertTrue(f.travel.isFinished());
        assertTrue(f.sync.isNavigationRequestCurrent(f.generation));
    }

    @Test void unsupportedGoalProximityIsNotArrival() {
        Fixture f = new Fixture();
        f.landBelowPlatform();
        f.sync.position.set(new Vector3d(200.5, 73, .5));
        f.sync.onGround.set(false);
        f.walk.setFinished(true);
        f.travel.onTick();
        assertTrue(f.travel.isFinished());
        assertNull(f.sync.getActiveGoal());
    }

    @Test void preparationReportsEquipmentAndDisabledModeSeparately() throws Exception {
        Fixture f = new Fixture();
        var prepare = TravelMovement.class.getDeclaredMethod("prepareLowFlight", MovementSync.class);
        prepare.setAccessible(true);
        assertEquals(false, prepare.invoke(f.travel, f.sync));
        assertEquals("equipment unavailable or unsafe", f.travel.lowFlightFailure());
        TravelMovement disabled = new TravelMovement(GOAL, f.generation, true, f,
                new FlightContextBuilder());
        assertEquals(false, prepare.invoke(disabled, f.sync));
        assertEquals("vanilla glide mode disabled", disabled.lowFlightFailure());
    }

    @Test void loadedNoCorridorPreparationReportsItsActualPlannerOutcome() throws Exception {
        Fixture f = new Fixture();
        f.equipped = true;
        var completed = new LowAltitudeFlightPlan.PlanFuture();
        var finish = LowAltitudeFlightPlan.PlanFuture.class.getDeclaredMethod("finish",
                LowAltitudeFlightPlan.Result.class, LowAltitudeFlightPlan.Status.class);
        finish.setAccessible(true);
        finish.invoke(completed, null, LowAltitudeFlightPlan.Status.NO_CORRIDOR);
        var plan = TravelMovement.class.getDeclaredField("lowPlan");
        plan.setAccessible(true);
        plan.set(f.travel, completed);
        var prepare = TravelMovement.class.getDeclaredMethod("prepareLowFlight", MovementSync.class);
        prepare.setAccessible(true);
        assertTrue(f.world.chunkLoaded(GOAL.x >> 4, GOAL.z >> 4));
        assertEquals(false, prepare.invoke(f.travel, f.sync));
        assertEquals("no safe corridor within geometric search budget", f.travel.lowFlightFailure());
        assertNull(plan.get(f.travel));
    }

    @Test void pendingPlannerReplacesStaleShoreInputsWithLedgeHold() throws Exception {
        Fixture f = new Fixture();
        f.equipped = true;
        f.sync.position.set(new Vector3d(245.5, 33, 20.5));
        var pending = new LowAltitudeFlightPlan.PlanFuture();
        var plan = TravelMovement.class.getDeclaredField("lowPlan");
        plan.setAccessible(true);
        plan.set(f.travel, pending);
        var prepare = TravelMovement.class.getDeclaredMethod("prepareLowFlight", MovementSync.class);
        prepare.setAccessible(true);
        for (int tick = 0; tick < 80; tick++) {
            f.sync.setInput(new xin.bbtt.physics.PlayerInput(true, false, false, false, true, false, true));
            assertEquals(true, prepare.invoke(f.travel, f.sync));
            assertEquals(new xin.bbtt.physics.PlayerInput(false, false, false, false, false, true, false),
                    f.sync.getInput(), "planning must not keep walking or jumping toward lava");
            assertEquals(0, f.walks);
        }
        f.travel.onStop();
        assertTrue(pending.isCancelled());
        assertEquals(xin.bbtt.physics.PlayerInput.NONE, f.sync.getInput());
    }

    /** Collision-backed shore: the last dry block is x=245, with lava at y=31 beyond it. */
    private static final class ShoreFixture implements TravelMovement.Legs {
        final World world = new World() {
            final xin.bbtt.Block.BlockState stone = new xin.bbtt.Block.BlockState(
                    "minecraft:stone", 1, java.util.Map.of(), "block", 1.5, true, "STONE");
            final xin.bbtt.Block.BlockState air = new xin.bbtt.Block.BlockState(
                    "minecraft:air", 0, java.util.Map.of(), "empty", 0, false, "AIR");
            final xin.bbtt.Block.BlockState lava = new xin.bbtt.Block.BlockState(
                    "minecraft:lava", 0, java.util.Map.of(), "empty", 0, false, "LAVA");
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public xin.bbtt.Block.BlockState getBlockStateAt(Vector3d p) {
                if (p.y < 30 || p.x < 246 && p.y < 33) return stone;
                return p.y < 32 ? lava : air;
            }
        };
        final MovementSync sync = new MovementSync() {
            @Override public World getWorld() { return world; }
            @Override public boolean isGliding() { return false; }
            @Override public boolean hasEquippedElytra() { return true; }
        };
        final Vector3i goal = new Vector3i(400, 33, 20);
        final long generation = sync.beginStaticNavigationRequest(goal);
        final TravelMovement travel;
        int plans;
        xin.bbtt.physics.PlayerPose pose = xin.bbtt.physics.PlayerPose.STANDING;

        ShoreFixture(boolean fly, double velocity) {
            travel = new TravelMovement(goal, generation, fly, this);
            sync.position.set(new Vector3d(245.5, 33, 20.5));
            sync.velocity.set(new Vector3d(velocity, 0, 0));
            sync.onGround.set(true);
            sync.yaw.set(-90f); // Stale forward input points straight out over the lava.
            sync.setInput(new xin.bbtt.physics.PlayerInput(true, false, false, false, true, false, true));
        }
        @Override public List<PathStep> plan(Vector3i destination) {
            assertEquals(goal, destination);
            plans++;
            return List.of(); // No safe walking route across the sea.
        }
        @Override public Movement walk(List<PathStep> path, Vector3i destination, boolean intermediate) {
            throw new AssertionError("No walking child may own controls without a safe route");
        }
        @Override public Movement flight() {
            throw new AssertionError("A pending plan must not launch a flight child");
        }
        void physicsTick(int tick) {
            var state = new xin.bbtt.physics.PhysicsState(sync.position.get(), sync.velocity.get(),
                    sync.onGround.get(), sync.fallDistance.get(), pose);
            var next = new xin.bbtt.physics.WalkingPhysics(sync.getInput(), sync.yaw.get(), true,
                    sync.getMovementParameters()).step(world, state, (double) 0.6f / 2,
                    (double) (float) pose.height());
            sync.position.set(next.position());
            sync.velocity.set(next.velocity());
            sync.onGround.set(next.onGround());
            sync.fallDistance.set(next.fallDistance());
            pose = next.pose();
            String diagnostic = "tick=" + tick + " position=" + next.position();
            assertEquals(33, next.position().y, 1e-6, "must remain on dry shore: " + diagnostic);
            assertTrue(next.position().x < 246 + (double) 0.6f / 2,
                    "player must retain shore support: " + diagnostic);
            assertTrue(world.isOnGround(next.position()), "lost physical support: " + diagnostic);
            assertTrue(next.position().y >= 32, "player body entered lava: " + diagnostic);
        }
    }

    @Test void pendingPlannerPhysicallyHoldsDryCoastForEightyTicks() throws Exception {
        exercisePendingShoreHold(0);
    }

    @Test void pendingPlannerBrakesExistingVelocityBeforeLeavingDryCoast() throws Exception {
        exercisePendingShoreHold(.45);
    }

    private void exercisePendingShoreHold(double velocity) throws Exception {
        ShoreFixture f = new ShoreFixture(true, velocity);
        var pending = new LowAltitudeFlightPlan.PlanFuture();
        var field = TravelMovement.class.getDeclaredField("lowPlan");
        field.setAccessible(true);
        field.set(f.travel, pending);
        var prepare = TravelMovement.class.getDeclaredMethod("prepareLowFlight", MovementSync.class);
        prepare.setAccessible(true);
        try {
            for (int tick = 0; tick < 80; tick++) {
                assertEquals(true, prepare.invoke(f.travel, f.sync));
                f.physicsTick(tick);
                assertFalse(f.travel.isFinished());
                assertFalse(pending.isDone(), "the injected planner must remain pending");
            }
            assertEquals(0, f.plans);
            if (velocity > 0) {
                assertTrue(f.sync.position.get().x > 245.5, "exercise real initial momentum");
                assertEquals(0, f.sync.velocity.get().x, 1e-6, "ledge hold must arrest momentum");
            }
        } finally {
            f.travel.onStop();
        }
        assertTrue(pending.isCancelled());
    }

    @Test void noWalkingFallbackRoutePhysicallyStaysOnDryCoast() {
        ShoreFixture f = new ShoreFixture(false, 0);
        try {
            f.travel.init();
            for (int tick = 0; tick < 80; tick++) {
                f.travel.onTick();
                f.physicsTick(tick);
            }
            assertEquals(1, f.plans);
            assertTrue(f.travel.isFinished());
            assertNull(f.sync.getActiveGoal(), "no safe fallback must abandon rather than claim arrival");
        } finally {
            f.travel.onStop();
        }
    }

    @Test void completedPlanCannotLaunchFromDifferentShorePosition() throws Exception {
        Fixture f = new Fixture();
        f.equipped = true;
        var completed = new LowAltitudeFlightPlan.PlanFuture();
        completed.complete(new LowAltitudeFlightPlan.Result(null,
                new Vector3d(245.5, 33, 20.5), new Vector3d(GOAL.x + .5, GOAL.y, GOAL.z + .5), 0, 0));
        var plan = TravelMovement.class.getDeclaredField("lowPlan");
        plan.setAccessible(true);
        plan.set(f.travel, completed);
        var prepare = TravelMovement.class.getDeclaredMethod("prepareLowFlight", MovementSync.class);
        prepare.setAccessible(true);
        assertEquals(false, prepare.invoke(f.travel, f.sync));
        assertEquals("launch position changed during planning", f.travel.lowFlightFailure());
        assertEquals(0, f.walks);
    }

    @Test void initialPlanningHoldDoesNotClearNewerNavigationInputsOnStop() {
        Fixture f = new Fixture();
        f.travel.init();
        assertTrue(f.sync.getInput().sneak());
        f.sync.beginStaticNavigationRequest(new Vector3i(300, 64, 0));
        var replacement = new xin.bbtt.physics.PlayerInput(true, false, false, false, false, false, true);
        f.sync.setInput(replacement);
        f.travel.onStop();
        assertSame(replacement, f.sync.getInput());
    }

    @Test void plannerDiagnosticsDoNotMislabelLoadedNoRouteAsMissingEquipment() {
        assertEquals("no safe corridor within geometric search budget",
                TravelMovement.lowPlanFailure(LowAltitudeFlightPlan.Status.NO_CORRIDOR));
        assertEquals("deployment blocked or unsafe",
                TravelMovement.lowPlanFailure(LowAltitudeFlightPlan.Status.DEPLOYMENT_REJECTED));
        assertEquals("flight planner busy", TravelMovement.lowPlanFailure(LowAltitudeFlightPlan.Status.BUSY));
    }
}
