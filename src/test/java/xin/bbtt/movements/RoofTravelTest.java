package xin.bbtt.movements;

import org.cloudburstmc.nbt.NbtMap;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.listeners.RegistryDataListener;
import xin.bbtt.movement.Movement;
import xin.bbtt.pathfinding.PathStep;
import xin.bbtt.world.World;

import java.util.List;
import java.util.OptionalDouble;

import static org.junit.jupiter.api.Assertions.*;

final class RoofTravelTest {
    private List<NbtMap> dimensions;
    private List<NbtMap> saved;
    private int index;
    @BeforeEach @SuppressWarnings("unchecked") void dimension() throws Exception {
        var field = RegistryDataListener.class.getDeclaredField("dimensionTypes");
        field.setAccessible(true);
        dimensions = (List<NbtMap>) field.get(null);
        saved = new java.util.ArrayList<>(dimensions);
        var current = RegistryDataListener.class.getDeclaredField("currentDimensionIndex");
        current.setAccessible(true);
        index = current.getInt(null);
        current.setInt(null, 0);
        dimensions.clear();
        dimensions.add(NbtMap.builder().putBoolean("has_ceiling", true).putInt("min_y", -32)
                .putInt("height", 288).putInt("logical_height", 96).build());
    }
    @AfterEach void restore() throws Exception {
        dimensions.clear(); dimensions.addAll(saved);
        var current = RegistryDataListener.class.getDeclaredField("currentDimensionIndex");
        current.setAccessible(true); current.setInt(null, index);
    }
    private static class RoofWorld extends World {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public OptionalDouble findHighestCollisionTop(double minX, double minY, double minZ,
                double maxX, double maxY, double maxZ) {
            return minY < 101 && maxY >= 101 ? OptionalDouble.of(101) : OptionalDouble.empty();
        }
        @Override public boolean isBoxColliding(double minX, double minY, double minZ,
                double maxX, double maxY, double maxZ) { return minY < 101 && maxY > 100; }
    }
    @Test void unloadedGoalColumnKeepsTheApproachAtRoofLevelUntilItLoads() {
        var loaded = new boolean[] {false};
        var roofWorld = new RoofWorld() {
            @Override public boolean chunkLoaded(int x, int z) { return x < 8 || loaded[0]; }
        };
        var sync = new MovementSync() {
            @Override public World getWorld() { return roofWorld; }
            @Override public boolean isGliding() { return true; }
        };
        sync.position.set(new Vector3d(.5, 120, .5)); sync.onGround.set(false);
        var flight = new ElytraFlyMovement(new Vector3d(200.5, 50, .5), sync.beginStaticNavigationRequest(new Vector3i(200, 50, 0)));
        flight.projectRoofTarget(sync);
        assertEquals(new Vector3d(200.5, 101, .5), flight.getTarget());
        assertFalse(flight.completedAboveRoof());
        loaded[0] = true;
        flight.projectRoofTarget(sync);
        assertEquals(new Vector3d(200.5, 101, .5), flight.getTarget());
    }
    @Test void roofTopIsOpenAndUndergroundTravelDoesNotApproachOrTakeOff() {
        var roofWorld = new RoofWorld();
        assertTrue(TravelMovement.isOpen(roofWorld, new Vector3d(.5, 101, .5)));
        assertFalse(TravelMovement.isOpen(roofWorld, new Vector3d(.5, 50, .5)));
        var sync = new MovementSync() { @Override public World getWorld() { return roofWorld; } };
        sync.position.set(new Vector3d(.5, 50, .5)); sync.onGround.set(true);
        var goal = new Vector3i(200, 50, 0);
        long generation = sync.beginStaticNavigationRequest(goal);
        var travel = new TravelMovement(goal, generation, true, new TravelMovement.Legs() {
            @Override public List<PathStep> plan(Vector3i destination) { assertEquals(goal, destination); return List.of(); }
            @Override public Movement walk(List<PathStep> path, Vector3i destination, boolean intermediate) {
                fail("empty path"); return null;
            }
            @Override public Movement flight() { fail("must not fly below roof"); return null; }
        });
        travel.onTick();
        assertTrue(travel.isFinished());
    }
    @Test void undergroundGoalProjectsToActualRoofAndTravelEndsWithoutWalkingOrRecovery() throws Exception {
        var roofWorld = new RoofWorld();
        var sync = new MovementSync() {
            @Override public World getWorld() { return roofWorld; }
            @Override public boolean isGliding() { return true; }
        };
        sync.position.set(new Vector3d(.5, 120, .5)); sync.onGround.set(false);
        var goal = new Vector3i(200, 50, 0);
        long generation = sync.beginStaticNavigationRequest(goal);
        var flight = new ElytraFlyMovement(new Vector3d(200.5, 50, .5), generation);
        assertTrue(RegistryDataListener.hasCeiling());
        assertTrue(TravelMovement.isOpen(roofWorld, sync.position.get()));
        assertNotNull(xin.bbtt.physics.ElytraFlightController.landingSite(roofWorld, 200.5, .5, 256));
        assertEquals(101, roofWorld.findHighestCollisionTop(200.2, 50, .2, 200.8, World.getMaxWorldY() + 1., .8).orElseThrow());
        assertEquals(101, xin.bbtt.physics.ElytraFlightController.landingSite(roofWorld, 200.5, .5, World.getMaxWorldY() + 1.).y);
        flight.projectRoofTarget(sync);
        assertEquals(new Vector3d(200.5, 101, .5), flight.getTarget());
        // Finish through the real leg's landing path; no equipment/physics mutation is needed.
        sync.position.set(new Vector3d(200.5, 101, .5)); sync.onGround.set(true);
        flight.onTick();
        assertTrue(flight.completedAboveRoof());
        var travel = new TravelMovement(goal, generation, true, new TravelMovement.Legs() {
            @Override public List<PathStep> plan(Vector3i destination) { fail("must not walk under roof"); return List.of(); }
            @Override public Movement walk(List<PathStep> path, Vector3i destination, boolean intermediate) { return null; }
            @Override public Movement flight() { return flight; }
        });
        var child = TravelMovement.class.getDeclaredField("child"); child.setAccessible(true); child.set(travel, flight);
        travel.onTick();
        assertTrue(travel.isFinished());
        assertNull(sync.getActiveGoal());
    }
}
