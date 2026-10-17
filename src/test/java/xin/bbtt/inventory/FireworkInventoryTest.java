package xin.bbtt.inventory;

import org.junit.jupiter.api.Test;
import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponents;
import org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponentTypes;
import org.geysermc.mcprotocollib.protocol.data.game.item.component.Fireworks;
import java.util.HashMap;
import java.util.List;
import static org.junit.jupiter.api.Assertions.*;

final class FireworkInventoryTest {
    private ItemStack rocket(int amount, Integer flight) {
        var patch = new DataComponents(new HashMap<>());
        if (flight != null) patch.put(DataComponentTypes.FIREWORKS, new Fireworks(flight, List.of()));
        return new ItemStack(ItemRegistry.Instance.getItem("firework_rocket").getId(), amount, patch);
    }

    @Test void expandsStackAmountsInHotbarOrderAndUsesDefaultFlightOne() {
        var manager = new InventoryManager();
        var items = new ItemStack[46];
        items[9] = rocket(64, 3);
        items[36] = rocket(2, null);
        items[37] = rocket(2, 0);
        items[38] = rocket(1, 3);
        items[39] = rocket(0, 3);
        items[40] = new ItemStack(ItemRegistry.Instance.getItem("stone").getId(), 64);
        items[44] = rocket(1, 2);
        items[45] = rocket(64, 3);
        manager.setContainerItems(0, items);
        assertArrayEquals(new int[]{1, 1, 0, 0, 3, 2}, manager.getHotbarFireworkFlightDurations());
        int[] snapshot = manager.getHotbarFireworkFlightDurations();
        snapshot[0] = 99;
        manager.setSlot(0, 36, rocket(1, 2));
        assertArrayEquals(new int[]{2, 0, 0, 3, 2}, manager.getHotbarFireworkFlightDurations());
    }

    @Test void handlesMissingShortInventoriesNullPatchesAndRemovedComponent() {
        var manager = new InventoryManager();
        assertArrayEquals(new int[0], manager.getHotbarFireworkFlightDurations());
        manager.setContainerItems(0, new ItemStack[10]);
        assertArrayEquals(new int[0], manager.getHotbarFireworkFlightDurations());
        var items = new ItemStack[38];
        items[36] = new ItemStack(ItemRegistry.Instance.getItem("firework_rocket").getId(), 1);
        var patch = new DataComponents(new HashMap<>());
        patch.getDataComponents().put(DataComponentTypes.FIREWORKS, null);
        items[37] = new ItemStack(ItemRegistry.Instance.getItem("firework_rocket").getId(), 1, patch);
        manager.setContainerItems(0, items);
        assertArrayEquals(new int[]{1, 0}, manager.getHotbarFireworkFlightDurations());
    }
}
