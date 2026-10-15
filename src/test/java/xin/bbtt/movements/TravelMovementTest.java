package xin.bbtt.movements;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class TravelMovementTest {
    @Test
    void decisionMatrix() {
        for (double distance : new double[]{0, 40, 96, 96.01, 400}) {
            for (boolean elytra : new boolean[]{false, true}) {
                for (boolean fireworks : new boolean[]{false, true}) {
                    for (boolean walk : new boolean[]{false, true}) {
                        for (boolean gliding : new boolean[]{false, true}) {
                            boolean expected = !walk && elytra && (gliding || distance > 96 && fireworks);
                            assertEquals(expected, TravelMovement.shouldFly(distance, elytra, fireworks, walk, gliding));
                        }
                    }
                }
            }
        }
    }

    @Test
    void distantAndLowRoofsBothBlockLaunch() {
        class RoofWorld extends World {
            double roofY = 80;
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) {
                return minY < roofY + 1 && maxY > roofY;
            }
        }
        RoofWorld world = new RoofWorld();
        Vector3d feet = new Vector3d(0.5, 64, 0.5);
        assertFalse(TravelMovement.isOpen(world, feet));
        world.roofY = 68;
        assertFalse(TravelMovement.isOpen(world, feet));
    }

    @Test
    void clearanceRequiresLoadedSkyAndLateralSpace() {
        class FakeWorld extends World {
            boolean loaded = true;
            boolean collision;
            @Override public boolean chunkLoaded(int x, int z) { return loaded; }
            @Override public boolean isBoxColliding(double minX, double minY, double minZ,
                                                     double maxX, double maxY, double maxZ) {
                assertEquals(3, maxX - minX);
                assertEquals(3, maxZ - minZ);
                assertEquals(World.getMaxWorldY() + 1.0, maxY);
                assertEquals(-59.99, minY, 0.00001);
                return collision;
            }
        }
        FakeWorld world = new FakeWorld();
        Vector3d feet = new Vector3d(0.5, -60, 0.5);
        assertTrue(TravelMovement.isOpen(world, feet));
        world.collision = true;
        assertFalse(TravelMovement.isOpen(world, feet));
        world.collision = false;
        world.loaded = false;
        assertFalse(TravelMovement.isOpen(world, feet));
    }
}
