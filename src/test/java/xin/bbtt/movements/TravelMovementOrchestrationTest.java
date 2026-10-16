package xin.bbtt.movements;

import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;
import xin.bbtt.pathfinding.BuiltinMovementType;
import xin.bbtt.pathfinding.Node;
import xin.bbtt.pathfinding.PathStep;
import xin.bbtt.world.World;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;

import static org.junit.jupiter.api.Assertions.*;

final class TravelMovementOrchestrationTest {
    private static final Vector3i GOAL = new Vector3i(200, 64, 0);

    private static final class LaunchWorld extends World {
        boolean open;
        xin.bbtt.Block.BlockState water;
        @Override public xin.bbtt.Block.BlockState getBlockStateAt(Vector3d p) {
            return water == null ? super.getBlockStateAt(p) : water;
        }
        boolean candidates = true;
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public OptionalDouble findHighestCollisionTop(double minX, double minY, double minZ,
                double maxX, double maxY, double maxZ) {
            return candidates && minY < 64 && maxY >= 64 ? OptionalDouble.of(64) : OptionalDouble.empty();
        }
        @Override public boolean isBoxColliding(double minX, double minY, double minZ,
                double maxX, double maxY, double maxZ) {
            return maxY - minY > 2 && !open && minX <= 0.5 && maxX >= 0.5
                    && minZ <= 0.5 && maxZ >= 0.5;
        }
    }

    private static final class Leg extends Movement {
        final String name;
        final List<String> events;
        Runnable tick = () -> {};
        Leg(String name, List<String> events) { this.name = name; this.events = events; }
        @Override public void init() { events.add(name); }
        @Override public void onTick() { tick.run(); }
        @Override public long getTime() { return -1; }
        @Override public void onStop() { finished = true; }
    }

    private static final class Fixture implements TravelMovement.Legs {
        final LaunchWorld world = new LaunchWorld();
        final MovementSync movementSync = new MovementSync() {
            @Override public World getWorld() { return Fixture.this.world; }
            @Override public boolean isGliding() { return gliding; }
        };
        boolean gliding;
        boolean approachPath = true;
        boolean finalPath = true;
        boolean partialFinalPath;
        int approachPlans;
        Runnable planning = () -> {};
        final List<String> events = new ArrayList<>();
        final List<Leg> children = new ArrayList<>();
        final long generation = movementSync.beginStaticNavigationRequest(GOAL);
        final TravelMovement travel = new TravelMovement(GOAL, generation, true, this);
        Fixture() { movementSync.position.set(new Vector3d(0.5, 64, 0.5)); }
        @Override public List<PathStep> plan(Vector3i destination) {
            planning.run();
            boolean terminal = destination.equals(GOAL);
            if (!terminal) approachPlans++;
            if (terminal ? !finalPath : !approachPath) return List.of();
            Node end = terminal && partialFinalPath ? new Node(10, 64, 0)
                    : new Node(destination.x, destination.y, destination.z);
            return List.of(new PathStep(new Node(0, 64, 0), BuiltinMovementType.WALK),
                    new PathStep(end, BuiltinMovementType.WALK));
        }
        Leg create(String name) { Leg leg = new Leg(name, events); children.add(leg); return leg; }
        @Override public Movement walk(List<PathStep> path, Vector3i destination, boolean intermediate) {
            return create(intermediate ? "approach" : "walk");
        }
        @Override public Movement flight() { return create("flight"); }
        Leg current() { return children.get(children.size() - 1); }
        void complete() { current().setFinished(true); travel.onTick(); }
    }

    @Test void waterHandoffStartsWalkingWithoutWaitingForGlidingMetadata() {
        Fixture fixture = new Fixture();
        fixture.gliding = true;
        fixture.movementSync.onGround.set(false);
        fixture.travel.onTick();
        assertEquals(List.of("flight"), fixture.events);
        fixture.world.water = java.util.stream.IntStream.range(0, xin.bbtt.Block.BlockStateParser.getBlockStateRegistrySize())
                .mapToObj(xin.bbtt.Block.BlockStateParser.Instance::parseStateId)
                .filter(block -> block.blockName().equals("water") && "0".equals(block.getProperty("level")))
                .findFirst().orElseThrow();
        fixture.complete();
        assertEquals(List.of("flight", "walk"), fixture.events);
        assertTrue(fixture.gliding);
        fixture.complete();
        assertTrue(fixture.travel.isFinished());
    }

    @Test void threeLegsRunInOrderAndFinish() {
        Fixture fixture = new Fixture();
        fixture.travel.onTick();
        assertEquals(List.of("approach"), fixture.events);
        fixture.world.open = true;
        fixture.complete();
        assertEquals(List.of("approach", "flight"), fixture.events);
        fixture.complete();
        assertEquals(List.of("approach", "flight", "walk"), fixture.events);
        fixture.complete();
        assertTrue(fixture.travel.isFinished());
        assertEquals(3, fixture.children.size());
    }

    @Test void openStartSkipsApproachAndFailedTakeoffIsNotRetried() {
        Fixture fixture = new Fixture();
        fixture.world.open = true;
        fixture.travel.onTick();
        assertEquals(List.of("flight"), fixture.events);
        fixture.complete(); // Flight terminates while still grounded: failed takeoff.
        fixture.complete();
        for (int tick = 0; tick < 10; tick++) fixture.travel.onTick();
        assertEquals(List.of("flight", "walk"), fixture.events);
        assertEquals(0, fixture.approachPlans);
    }

    @Test void failedApproachFallsBackWithoutFlight() {
        Fixture fixture = new Fixture();
        fixture.travel.onTick();
        fixture.complete(); // Still beneath the roof.
        assertEquals(List.of("approach", "walk"), fixture.events);
    }

    @Test void noCandidateOrNoApproachPathFallsBackWithBoundedWork() {
        for (boolean candidates : new boolean[]{false, true}) {
            Fixture fixture = new Fixture();
            fixture.world.candidates = candidates;
            fixture.approachPath = false;
            fixture.travel.onTick();
            assertEquals(List.of("walk"), fixture.events);
            assertEquals(candidates ? TravelMovement.MAX_TAKEOFF_PATH_ATTEMPTS : 0, fixture.approachPlans);
        }
    }

    @Test void failedFlightWaitsForRealLandingBeforeWalking() {
        Fixture fixture = new Fixture();
        fixture.world.open = true;
        fixture.travel.onTick();
        fixture.movementSync.onGround.set(false);
        fixture.gliding = true;
        fixture.complete();
        fixture.travel.onTick();
        assertEquals(List.of("flight"), fixture.events);
        fixture.movementSync.onGround.set(true);
        fixture.travel.onTick();
        assertEquals(List.of("flight"), fixture.events);
        fixture.gliding = false;
        fixture.travel.onTick();
        assertEquals(List.of("flight", "walk"), fixture.events);
    }

    @Test void airborneGotoContinuesFlightEvenWithoutOpenLaunchColumn() {
        Fixture fixture = new Fixture();
        fixture.gliding = true;
        fixture.movementSync.onGround.set(false);
        fixture.travel.onTick();
        assertEquals(List.of("flight"), fixture.events);
        assertEquals(0, fixture.approachPlans);
    }

    @Test void fallingWithoutGlideWaitsForGroundBeforeStarting() {
        Fixture fixture = new Fixture();
        fixture.world.open = true;
        fixture.movementSync.onGround.set(false);
        fixture.travel.onTick();
        assertTrue(fixture.events.isEmpty());
        fixture.movementSync.onGround.set(true);
        fixture.travel.onTick();
        assertEquals(List.of("flight"), fixture.events);
    }

    @Test void cancellationDuringFinalPlanningCannotStartWalk() {
        for (boolean replace : new boolean[]{false, true}) {
            Fixture fixture = new Fixture();
            fixture.world.open = true;
            fixture.travel.onTick();
            fixture.planning = replace ? () -> fixture.movementSync.beginNavigationRequest()
                    : fixture.travel::onStop;
            fixture.complete();
            assertEquals(List.of("flight"), fixture.events);
        }
    }

    @Test void missingFinalPathAbandonsGoal() {
        Fixture fixture = new Fixture();
        fixture.world.candidates = false;
        fixture.finalPath = false;
        fixture.travel.onTick();
        assertTrue(fixture.travel.isFinished());
        assertNull(fixture.movementSync.getActiveGoal());
        assertTrue(fixture.events.isEmpty());
    }

    @Test void partialFinalPathStartsWalkingWithoutCompletingGoal() {
        Fixture fixture = new Fixture();
        fixture.world.candidates = false;
        fixture.partialFinalPath = true;
        fixture.travel.onTick();
        assertEquals(List.of("walk"), fixture.events);
        assertFalse(fixture.travel.isFinished());
        assertEquals(GOAL, fixture.movementSync.getActiveGoal());
    }

    @Test void cancelOrReplaceAtEveryStagePreventsNextLeg() {
        for (boolean replace : new boolean[]{false, true}) {
            for (int stage = 0; stage < 5; stage++) {
                Fixture fixture = new Fixture();
                if (stage >= 1) fixture.travel.onTick();
                if (stage >= 2) { fixture.world.open = true; fixture.complete(); }
                if (stage >= 3) {
                    fixture.movementSync.onGround.set(false);
                    fixture.gliding = true;
                    fixture.complete();
                }
                if (stage >= 4) {
                    fixture.movementSync.onGround.set(true);
                    fixture.gliding = false;
                    fixture.travel.onTick();
                }
                List<String> before = List.copyOf(fixture.events);
                if (replace) fixture.movementSync.beginStaticNavigationRequest(new Vector3i(300, 64, 0));
                else fixture.travel.onStop();
                fixture.movementSync.onGround.set(true);
                fixture.gliding = false;
                fixture.world.open = true;
                if (!fixture.children.isEmpty()) fixture.current().setFinished(true);
                fixture.travel.onTick();
                assertEquals(before, fixture.events, "stage=" + stage + ", replace=" + replace);
            }
        }
    }

    @Test void cancellationDuringChildTickOrPlanningCannotStartAnotherLeg() {
        for (boolean replace : new boolean[]{false, true}) {
            Fixture fixture = new Fixture();
            Runnable cancel = replace ? () -> fixture.movementSync.beginNavigationRequest() : fixture.travel::onStop;
            fixture.travel.onTick();
            fixture.world.open = true;
            fixture.current().tick = () -> { cancel.run(); fixture.current().setFinished(true); };
            fixture.travel.onTick();
            assertEquals(List.of("approach"), fixture.events);

            Fixture planningFixture = new Fixture();
            planningFixture.planning = replace ? () -> planningFixture.movementSync.beginNavigationRequest()
                    : planningFixture.travel::onStop;
            planningFixture.travel.onTick();
            assertTrue(planningFixture.events.isEmpty());
        }
    }
}
