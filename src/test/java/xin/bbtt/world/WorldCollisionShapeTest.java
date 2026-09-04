package xin.bbtt.world;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.OptionalDouble;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;

final class WorldCollisionShapeTest {
    private static final int AIR = 0;
    private static final int OPEN_NORTH_LEFT_JUNGLE_DOOR = 13995;
    private static final int BOTTOM_OAK_SLAB = 13131;
    private static final int TALL_OAK_FENCE = 6764;
    private static final int WEST_EXTENDING_PISTON_HEAD = 2075;

    private static World singleBlock(int stateId) {
        return new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                return x == 0 && y == 0 && z == 0 ? stateId : AIR;
            }
        };
    }

    @Test
    void openDoorCentreIsClearButRotatedPanelStillCollides() {
        World world = singleBlock(OPEN_NORTH_LEFT_JUNGLE_DOOR);

        assertFalse(world.isBoxColliding(0.20, 0.0, 0.20, 0.80, 1.8, 0.80));
        assertTrue(world.isBoxColliding(-0.10, 0.0, 0.20, 0.20, 1.8, 0.80));
    }

    @Test
    void sourceBlocksAreConsideredWhenTheirShapesExtendOutsideTheirCell() {
        World fenceWorld = singleBlock(TALL_OAK_FENCE);
        assertTrue(fenceWorld.isBoxColliding(0.4, 1.1, 0.4, 0.6, 1.4, 0.6),
            "fence shape from y=0 extends into the y=1 cell");

        World pistonWorld = singleBlock(WEST_EXTENDING_PISTON_HEAD);
        assertTrue(pistonWorld.isBoxColliding(-0.20, 0.4, 0.4, -0.10, 0.6, 0.6),
            "piston head in x=0 extends into the x=-1 cell");
    }

    @Test
    void supportSurfaceUsesActualShapeTopInsteadOfIntegerBlockHeight() {
        World world = singleBlock(BOTTOM_OAK_SLAB);

        OptionalDouble support = world.findHighestCollisionTop(
            -0.2, -0.1, -0.2,
             0.8,  1.0,  0.8);

        assertTrue(support.isPresent());
        assertEquals(0.5, support.getAsDouble(), 1e-9);
        assertTrue(world.isOnGround(new Vector3d(0.5, 0.5, 0.5)),
            "feet on the slab's real top must be grounded");
        assertFalse(world.isOnGround(new Vector3d(0.5, 1.0, 0.5)),
            "air above a bottom slab is not a support surface");
    }
}
