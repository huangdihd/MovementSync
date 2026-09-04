package xin.bbtt.tasks;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

final class VerticalCollisionResolverTest {
    private static final int AIR = 0;
    private static final int BOTTOM_OAK_SLAB = 13131;

    private static World slabWorld() {
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
                return x == 0 && y == 0 && z == 0 ? BOTTOM_OAK_SLAB : AIR;
            }
        };
    }

    @Test
    void downwardMotionLandsOnExactCollisionTop() {
        World world = slabWorld();
        Vector3d current = new Vector3d(0.5, 0.7, 0.5);

        assertEquals(0.5,
            VerticalCollisionResolver.resolveDownwardY(world, current, 0.299, -0.3), 1e-9);
        assertEquals(0.6,
            VerticalCollisionResolver.resolveDownwardY(world, current, 0.299, -0.1), 1e-9);
    }
}
