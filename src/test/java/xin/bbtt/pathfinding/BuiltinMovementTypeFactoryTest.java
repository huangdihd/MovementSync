package xin.bbtt.pathfinding;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.lang.reflect.Field;
import java.util.Map;
import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.MovementSync;
import xin.bbtt.inventory.ItemRegistry;
import xin.bbtt.movements.PillarMovement;
import xin.bbtt.world.World;

final class BuiltinMovementTypeFactoryTest {
    private static final BlockState STONE = new BlockState(
        "minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState(
        "minecraft:air", 0, Map.of(), "empty", 0.0, false, "AIR");

    @AfterEach
    void resetSingleton() {
        MovementSync.INSTANCE = null;
    }

    @Test
    void pillarFactoryDoesNotInsertMovementOrSwitchSlots() throws Exception {
        MovementSync movementSync = new MovementSync();
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public BlockState getBlockStateAt(Vector3d position) {
                return Math.floor(position.y) == 63 ? STONE : AIR;
            }
        });
        ItemStack[] inventory = new ItemStack[45];
        inventory[36] = new ItemStack(ItemRegistry.Instance.getItem("stone").getId(), 64);
        movementSync.getInventoryManager().setContainerItems(0, inventory);
        movementSync.getInventoryManager().setHeldSlot(1);

        xin.bbtt.movement.Movement result = BuiltinMovementType.PILLAR.createMovement(
            new Node(0, 64, 0), new Node(0, 65, 0));

        assertInstanceOf(PillarMovement.class, result);
        assertNull(movementSync.getMovementController().getCurrentMovement());
        assertEquals(1, movementSync.getInventoryManager().getHeldSlot(),
            "factory creation must not switch the held slot");
    }
}
