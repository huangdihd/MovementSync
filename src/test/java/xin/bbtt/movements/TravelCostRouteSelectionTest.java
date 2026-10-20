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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

final class TravelCostRouteSelectionTest {
    private static final Vector3i GOAL = new Vector3i(200, 64, 0);

    private static final class OpenWorld extends World {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public boolean isBoxColliding(double minX, double minY, double minZ,
                double maxX, double maxY, double maxZ) { return false; }
        @Override public OptionalDouble findHighestCollisionTop(double minX, double minY, double minZ,
                double maxX, double maxY, double maxZ) { return OptionalDouble.of(64); }
    }

    private static final class Leg extends Movement {
        private final String name;
        private final List<String> events;
        private Leg(String name, List<String> events) { this.name = name; this.events = events; }
        @Override public void init() { events.add(name); }
        @Override public void onTick() {}
        @Override public long getTime() { return -1; }
        @Override public void onStop() { finished = true; }
    }

    private static final class Fixture implements TravelMovement.Legs {
        final OpenWorld world = new OpenWorld();
        final MovementSync sync = new MovementSync() {
            @Override public World getWorld() { return world; }
            @Override public boolean hasEquippedElytra() { return true; }
        };
        final List<String> events = new ArrayList<>();
        final List<Leg> children = new ArrayList<>();
        final long generation = sync.beginStaticNavigationRequest(GOAL);
        final TravelMovement travel;
        boolean completeWalk = true;
        double walkCost = 100;
        double flightCost = 50;
        int walkingPlans;
        int flightPreviews;

        Fixture(boolean allowFlight) {
            sync.position.set(new Vector3d(.5, 64, .5));
            sync.onGround.set(true);
            travel = new TravelMovement(GOAL, generation, allowFlight, this,
                    new xin.bbtt.flight.FlightContextBuilder().addStandard(), true);
        }

        @Override public List<PathStep> plan(Vector3i destination) {
            walkingPlans++;
            if (!completeWalk) return List.of();
            return List.of(new PathStep(new Node(0, 64, 0), BuiltinMovementType.WALK),
                    new PathStep(new Node(destination.x, destination.y, destination.z), BuiltinMovementType.WALK));
        }
        @Override public double walkingCost(List<PathStep> path) { return walkCost; }
        @Override public double flightCost(Vector3i destination) {
            flightPreviews++;
            return flightCost;
        }
        @Override public Movement walk(List<PathStep> path, Vector3i destination, boolean intermediate) {
            return child(intermediate ? "approach" : "walk");
        }
        @Override public Movement flight() { return child("flight"); }
        @Override public xin.bbtt.inventory.InventoryManager.ElytraAction prepareFlight() {
            return xin.bbtt.inventory.InventoryManager.ElytraAction.READY;
        }
        @Override public boolean hasFlightBudget() { return true; }

        private Leg child(String name) {
            Leg child = new Leg(name, events);
            children.add(child);
            return child;
        }
        void finishAt(Vector3d position) {
            sync.position.set(position);
            sync.onGround.set(true);
            children.get(children.size() - 1).setFinished(true);
            travel.onTick();
        }
    }

    @Test void cheaperCompleteWalkingRouteWins() {
        Fixture fixture = new Fixture(true);
        fixture.walkCost = 40;
        fixture.flightCost = 80;
        fixture.travel.onTick();
        assertEquals(List.of("walk"), fixture.events);
    }

    @Test void cheaperFlightWinsEvenWhenWalkingCanReachGoal() {
        Fixture fixture = new Fixture(true);
        fixture.walkCost = 200;
        fixture.flightCost = 60;
        fixture.travel.onTick();
        assertEquals(List.of("flight"), fixture.events);
    }

    @Test void missingWalkingRouteMakesFlightTheOnlyFiniteCandidate() {
        Fixture fixture = new Fixture(true);
        fixture.completeWalk = false;
        fixture.travel.onTick();
        assertEquals(List.of("flight"), fixture.events);
    }

    @Test void unavailableFlightLeavesWalkingCandidate() {
        Fixture fixture = new Fixture(true);
        fixture.flightCost = Double.POSITIVE_INFINITY;
        fixture.travel.onTick();
        assertEquals(List.of("walk"), fixture.events);
    }

    @Test void explicitWalkDisablesCheaperFlightCandidate() {
        Fixture fixture = new Fixture(false);
        fixture.walkCost = 200;
        fixture.flightCost = 1;
        fixture.travel.onTick();
        assertEquals(List.of("walk"), fixture.events);
        assertEquals(0, fixture.flightPreviews);
    }

    @Test void failedTakeoffAtSamePositionDoesNotImmediatelyRetryFlight() {
        Fixture fixture = new Fixture(true);
        fixture.completeWalk = false;
        fixture.travel.onTick();
        fixture.finishAt(new Vector3d(.5, 64, .5));
        assertEquals(List.of("flight"), fixture.events);
        assertEquals(1, fixture.flightPreviews);
        assertTrue(fixture.travel.isFinished());
    }

    @Test void failedTakeoffAfterApproachUsesActualLaunchPositionForRetrySuppression() throws Exception {
        Fixture fixture = new Fixture(true);
        var selected = TravelMovement.class.getDeclaredField("routeSelected");
        selected.setAccessible(true);
        selected.setBoolean(fixture.travel, true);
        var staleStart = TravelMovement.class.getDeclaredField("flightStartPosition");
        staleStart.setAccessible(true);
        staleStart.set(fixture.travel, new Vector3d(.5, 64, .5));

        fixture.sync.position.set(new Vector3d(10.5, 64, .5));
        var startFlight = TravelMovement.class.getDeclaredMethod("startFlight");
        startFlight.setAccessible(true);
        startFlight.invoke(fixture.travel);
        fixture.finishAt(new Vector3d(10.5, 64, .5));

        assertEquals(List.of("flight", "walk"), fixture.events);
        assertEquals(0, fixture.flightPreviews,
                "an unchanged launch position must block the repeated flight preview");
    }

    @Test void intermediateLandingReplansAndCanChooseAnotherFlightLeg() {
        Fixture fixture = new Fixture(true);
        fixture.completeWalk = false;
        fixture.travel.onTick();
        assertEquals(List.of("flight"), fixture.events);
        fixture.finishAt(new Vector3d(150.5, 64, .5));
        assertEquals(List.of("flight", "flight"), fixture.events);
        assertFalse(fixture.travel.isFinished());
    }

    @Test void pluginPreviewFailureIsAnUnavailableCandidateNotAnEscapedException() {
        OpenWorld world = new OpenWorld();
        MovementSync sync = new MovementSync() {
            @Override public World getWorld() { return world; }
        };
        sync.position.set(new Vector3d(.5, 64, .5));
        sync.onGround.set(true);
        var failingMode = new xin.bbtt.flight.FlightMode() {
            @Override public boolean isAvailable(xin.bbtt.flight.FlightContext context) {
                throw new IllegalStateException("preview boom");
            }
            @Override public List<? extends xin.bbtt.flight.FlightAction> candidates(
                    xin.bbtt.flight.FlightContext context) { return List.of(); }
            @Override public xin.bbtt.flight.Trajectory predict(xin.bbtt.flight.FlightContext context,
                    xin.bbtt.flight.FlightAction action) { throw new AssertionError(); }
            @Override public void execute(xin.bbtt.flight.FlightContext context,
                    xin.bbtt.flight.FlightAction action, MovementSync movementSync) {}
        };
        long generation = sync.beginStaticNavigationRequest(GOAL);
        TravelMovement travel = new TravelMovement(GOAL, generation, true,
                new xin.bbtt.flight.FlightContextBuilder().addMode(failingMode));

        assertFalse(travel.hasFiniteFlightCandidate());
        assertTrue(travel.lowFlightFailure().contains("preview boom"));
    }

    @Test void landingOnlyPluginIsNotAFeasibleDeparture() {
        OpenWorld world = new OpenWorld();
        MovementSync sync = new MovementSync() {
            @Override public World getWorld() { return world; }
        };
        sync.position.set(new Vector3d(.5, 64, .5));
        sync.onGround.set(true);
        var landingOnly = new xin.bbtt.flight.FlightMode() {
            final xin.bbtt.flight.FlightAction action = new xin.bbtt.flight.FlightAction() {};
            @Override public List<? extends xin.bbtt.flight.FlightAction> candidates(
                    xin.bbtt.flight.FlightContext context) { return List.of(); }
            @Override public xin.bbtt.flight.Trajectory predict(xin.bbtt.flight.FlightContext context,
                    xin.bbtt.flight.FlightAction candidate) { throw new AssertionError(); }
            @Override public xin.bbtt.flight.Trajectory previewLanding(
                    xin.bbtt.flight.FlightContext context) {
                return new xin.bbtt.flight.Trajectory(context.state(), List.of(
                        new xin.bbtt.physics.PhysicsResult(context.state().position(), new Vector3d(),
                                true, false, false)), action);
            }
            @Override public Movement createLanding(xin.bbtt.flight.FlightContext context) {
                return new Leg("landing", new ArrayList<>());
            }
            @Override public void execute(xin.bbtt.flight.FlightContext context,
                    xin.bbtt.flight.FlightAction candidate, MovementSync movementSync) {}
        };
        long generation = sync.beginStaticNavigationRequest(GOAL);
        TravelMovement travel = new TravelMovement(GOAL, generation, true,
                new xin.bbtt.flight.FlightContextBuilder().addMode(landingOnly));

        assertFalse(travel.hasFiniteFlightCandidate());
    }
}
