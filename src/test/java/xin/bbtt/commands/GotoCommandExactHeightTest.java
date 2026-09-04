package xin.bbtt.commands;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.pathfinding.Node;
import xin.bbtt.world.World;

final class GotoCommandExactHeightTest {
    private static final int AIR = 0;
    private static final int FOUR_LAYER_SNOW = 6721;

    @Test
    void acceptsLoadedGoalSupportedByExactNonSolidCollisionShape() {
        World world = new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                return x == 0 && y == 0 && z == 0 ? FOUR_LAYER_SNOW : AIR;
            }
        };

        assertTrue(GotoCommandExecutor.isLoadedGoalStandable(world, new Node(0, 1, 0)));
    }
}
