package xin.bbtt.physics;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import org.cloudburstmc.nbt.NbtMap;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.listeners.RegistryDataListener;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class CeilingCollisionTest {
    private List<NbtMap> dimensions;
    private List<NbtMap> saved;
    private Field index;
    private int savedIndex;

    @BeforeEach @SuppressWarnings("unchecked") void setup() throws Exception {
        Field types = RegistryDataListener.class.getDeclaredField("dimensionTypes");
        types.setAccessible(true);
        dimensions = (List<NbtMap>) types.get(null);
        saved = new ArrayList<>(dimensions);
        index = RegistryDataListener.class.getDeclaredField("currentDimensionIndex");
        index.setAccessible(true);
        savedIndex = index.getInt(null);
        dimension(true);
    }

    private void dimension(boolean ceiling) throws Exception {
        dimensions.clear();
        dimensions.add(NbtMap.builder().putInt("min_y", -32).putInt("height", 288)
                .putInt("logical_height", 96).putBoolean("has_ceiling", ceiling).build());
        index.setInt(null, 0);
    }

    @AfterEach void restore() throws Exception {
        dimensions.clear();
        dimensions.addAll(saved);
        index.setInt(null, savedIndex);
    }

    private static final class Terrain extends World {
        boolean loaded = true;
        double roof = 173.5;
        int queries;
        @Override public boolean chunkLoaded(int x, int z) { return loaded; }
        @Override public OptionalDouble findHighestCollisionTop(double x, double y, double z,
                double X, double Y, double Z) {
            queries++;
            double top = roof <= Y ? roof : 40;
            return top >= y && top <= Y ? OptionalDouble.of(top) : OptionalDouble.empty();
        }
        @Override public boolean isBoxColliding(double x, double y, double z, double X, double Y, double Z) {
            return y < roof && Y > roof - 1;
        }
    }

    @Test void usesActualFractionalCollisionFaceNotLogicalHeight() {
        Terrain world = new Terrain();
        assertTrue(RegistryDataListener.hasCeiling());
        assertEquals(96, RegistryDataListener.getLogicalHeight());
        assertFalse(CeilingCollision.isAboveRoof(world, new Vector3d(8, 100, 8)));
        assertTrue(CeilingCollision.isAboveRoof(world, new Vector3d(8, 173.5, 8)));
        Vector3d target = new Vector3d(8, 40, 8);
        assertTrue(CeilingCollision.hasRoofAbove(world, target));
        assertEquals(new Vector3d(8, 173.5, 8), CeilingCollision.projectLandingTarget(world, target));
        assertEquals(40, target.y, "projection must not mutate the requested destination");
    }

    @Test void unknownAndNonCeilingColumnsDoNotInventRoofs() throws Exception {
        Terrain world = new Terrain();
        world.loaded = false;
        assertFalse(CeilingCollision.isAboveRoof(world, new Vector3d(8, 200, 8)));
        assertNull(CeilingCollision.projectLandingTarget(world, new Vector3d(8, 40, 8)));
        assertEquals(0, world.queries);
        world.loaded = true;
        dimension(false);
        assertFalse(CeilingCollision.hasRoofAbove(world, new Vector3d(8, 40, 8)));
        assertEquals(0, world.queries);
    }

    @Test void registryIndexAndMissingDataAreHandledWithoutGuessing() throws Exception {
        dimensions.add(null);
        index.setInt(null, 1);
        assertFalse(RegistryDataListener.hasCeiling());
        assertEquals(384, RegistryDataListener.getLogicalHeight());
        index.setInt(null, 0);
        assertTrue(RegistryDataListener.hasCeiling());
        assertEquals(96, RegistryDataListener.getLogicalHeight());
    }

    @Test void routeInsideRoofDoesNotAimAboveItButAboveRoofRouteDoes() {
        Terrain world = new Terrain();
        assertEquals(56, FlightSelector.routeAltitude(world, new Vector3d(8, 80, 8), new Vector3d(8, 40, 24)));
        assertEquals(189.5, FlightSelector.routeAltitude(world, new Vector3d(8, 200, 8), new Vector3d(8, 40, 24)));
    }

    @Test void noCeilingRouteRetainsLegacyBits() throws Exception {
        dimension(false);
        Terrain world = new Terrain();
        double actual = FlightSelector.routeAltitude(world, new Vector3d(8, 80, 8), new Vector3d(8, 40, 24));
        double legacy = Math.min(World.getMaxWorldY() - VanillaGlideMode.HEIGHT - 2, world.roof + 16);
        assertEquals(Double.doubleToRawLongBits(legacy), Double.doubleToRawLongBits(actual));
    }
}
