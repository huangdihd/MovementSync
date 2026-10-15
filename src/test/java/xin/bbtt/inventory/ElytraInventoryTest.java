package xin.bbtt.inventory;

import org.junit.jupiter.api.Test;
import org.geysermc.mcprotocollib.network.packet.Packet;
import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponents;
import org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponentTypes;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundSetCarriedItemPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.inventory.ServerboundContainerClickPacket;
import xin.bbtt.physics.ElytraDurability;
import java.util.ArrayList;
import java.util.HashMap;
import static org.junit.jupiter.api.Assertions.*;

class ElytraInventoryTest {
    private ItemStack elytra(int damage) {
        var patch = new DataComponents(new HashMap<>());
        patch.put(DataComponentTypes.DAMAGE, damage);
        return new ItemStack(ItemRegistry.Instance.getItem("elytra").getId(), 1, patch);
    }
    @Test void budgetOnlyCountsChestAndHotbarAndUsesServerDamage() {
        var manager = new InventoryManager();
        var items = new ItemStack[46];
        items[6] = elytra(422); items[9] = elytra(0); items[36] = elytra(400);
        manager.setContainerItems(0, items);
        assertEquals(40, manager.remainingFlightSeconds());
        assertTrue(manager.canStartFlight(100));
        assertFalse(manager.canStartFlight(1000));
        manager.setSlot(0, 36, elytra(431));
        assertEquals(9, manager.remainingFlightSeconds());
        assertFalse(manager.canStartFlight(0));
    }
    @Test void airborneSwapIsRightClickThenRestoreAndWaitsForBothServerSlots() {
        var manager = new InventoryManager();
        var items = new ItemStack[46]; items[6] = elytra(422); items[37] = elytra(0);
        manager.setContainerItems(0, items); manager.setHeldSlot(4);
        var packets = new ArrayList<Packet>();
        assertEquals(InventoryManager.ElytraAction.WAITING, manager.tickElytraSwap(false, packets::add, () -> 7, 1, 2));
        assertEquals(3, packets.size());
        assertEquals(1, ((ServerboundSetCarriedItemPacket) packets.get(0)).getSlot());
        assertInstanceOf(ServerboundUseItemPacket.class, packets.get(1));
        assertEquals(4, ((ServerboundSetCarriedItemPacket) packets.get(2)).getSlot());
        assertEquals(422, manager.getElytraDurability().damage());
        manager.setSlot(0, 6, items[37]);
        assertEquals(InventoryManager.ElytraAction.WAITING, manager.tickElytraSwap(false, packets::add, () -> 8, 0, 0));
        manager.setSlot(0, 37, items[6]);
        assertEquals(InventoryManager.ElytraAction.READY, manager.tickElytraSwap(false, packets::add, () -> 8, 0, 0));
        assertEquals(3, packets.size());
    }
    @Test void preparationIsGroundOnlyAndDoesNotPredictInventory() {
        var manager = new InventoryManager(); var items = new ItemStack[46];
        items[6] = elytra(0); items[9] = elytra(0); manager.setContainerItems(0, items);
        var packets = new ArrayList<Packet>();
        assertEquals(InventoryManager.ElytraAction.LAND, manager.prepareElytra(false, packets::add));
        assertTrue(packets.isEmpty());
        assertEquals(InventoryManager.ElytraAction.WAITING, manager.prepareElytra(true, packets::add));
        assertInstanceOf(ServerboundContainerClickPacket.class, packets.get(0));
        assertEquals(431, manager.remainingFlightSeconds());
        manager.setSlot(0, 36, items[9]); manager.setSlot(0, 9, null);
        assertEquals(InventoryManager.ElytraAction.READY, manager.prepareElytra(true, packets::add));
        assertEquals(862, manager.remainingFlightSeconds());
    }
    @Test void rejectedSwapTimesOutWithoutRepeating() {
        var manager = new InventoryManager(); var items = new ItemStack[46];
        items[6] = elytra(422); items[36] = elytra(0); manager.setContainerItems(0, items);
        var packets = new ArrayList<Packet>();
        for (int i = 0; i < 40; i++) assertEquals(InventoryManager.ElytraAction.WAITING,
                manager.tickElytraSwap(false, packets::add, () -> 1, 0, 0));
        assertEquals(InventoryManager.ElytraAction.LAND, manager.tickElytraSwap(false, packets::add, () -> 1, 0, 0));
        assertEquals(InventoryManager.ElytraAction.LAND, manager.tickElytraSwap(false, packets::add, () -> 1, 0, 0));
        assertEquals(3, packets.size());
    }
    @Test void playerInventoryPacketSynchronizesChestAndHotbarIndices() {
        var sync = new xin.bbtt.MovementSync();
        sync.getInventoryManager().setContainerItems(0, new ItemStack[46]);
        var listener = new xin.bbtt.listeners.InventoryPacketListener();
        listener.packetReceived(null, new org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.inventory.ClientboundSetPlayerInventoryPacket(38, elytra(400)));
        listener.packetReceived(null, new org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.inventory.ClientboundSetPlayerInventoryPacket(0, elytra(400)));
        assertEquals(400, sync.getInventoryManager().getElytraDurability().damage());
        assertEquals(62, sync.getInventoryManager().remainingFlightSeconds());
    }
    @Test void bindingChestEquipmentExcludesUnusableReservesAndSendsNoSwap() {
        var ids = xin.bbtt.listeners.RegistryDataListener.getEnchantmentIds();
        Integer previous = ids.put("minecraft:binding_curse", 987);
        try {
            var manager = new InventoryManager();
            var items = new ItemStack[46];
            var patch = new DataComponents(new HashMap<>());
            patch.put(DataComponentTypes.ENCHANTMENTS,
                    new org.geysermc.mcprotocollib.protocol.data.game.item.component.ItemEnchantments(java.util.Map.of(987, 1)));
            items[6] = new ItemStack(ItemRegistry.Instance.getItem("diamond_chestplate").getId(), 1, patch);
            items[36] = elytra(0);
            manager.setContainerItems(0, items);
            var packets = new ArrayList<Packet>();
            assertEquals(0, manager.remainingFlightSeconds());
            assertEquals(InventoryManager.ElytraAction.LAND, manager.tickElytraSwap(false, packets::add, () -> 1, 0, 0));
            assertTrue(packets.isEmpty());
            var boundElytra = elytra(422);
            boundElytra.getDataComponentsPatch().put(DataComponentTypes.ENCHANTMENTS, patch.get(DataComponentTypes.ENCHANTMENTS));
            manager.setSlot(0, 6, boundElytra);
            assertEquals(9, manager.remainingFlightSeconds());
            manager.setSlot(0, 6, elytra(422));
            assertEquals(440, manager.remainingFlightSeconds());
        } finally {
            if (previous == null) ids.remove("minecraft:binding_curse");
            else ids.put("minecraft:binding_curse", previous);
        }
    }
    @Test void sessionResetClearsPendingAndFailedSwapsAndOldInventory() {
        for (boolean timeout : new boolean[] {false, true}) {
            var manager = new InventoryManager();
            var items = new ItemStack[46]; items[6] = elytra(422); items[36] = elytra(0);
            manager.setContainerItems(0, items);
            var packets = new ArrayList<Packet>();
            manager.tickElytraSwap(false, packets::add, () -> 1, 0, 0);
            if (timeout) for (int i = 0; i < 40; i++) manager.tickElytraSwap(false, packets::add, () -> 1, 0, 0);
            manager.setContainerItems(3, new ItemStack[9]);
            manager.setCurrentContainerId(3); manager.setCurrentStateId(20); manager.setHeldSlot(4);
            manager.resetSession();
            assertNull(manager.getInventory());
            assertNull(manager.getContainer(3));
            assertEquals(0, manager.getCurrentContainerId());
            assertEquals(0, manager.getCurrentStateId());
            assertEquals(0, manager.getHeldSlot());
            assertEquals(0, manager.remainingFlightSeconds());
            manager.setContainerItems(0, items);
            packets.clear();
            assertEquals(InventoryManager.ElytraAction.WAITING, manager.tickElytraSwap(false, packets::add, () -> 2, 0, 0));
            assertEquals(3, packets.size());
            manager.setSlot(0, 6, items[36]); manager.setSlot(0, 36, items[6]);
            assertEquals(InventoryManager.ElytraAction.READY, manager.tickElytraSwap(false, packets::add, () -> 3, 0, 0));
        }
    }
    @Test void vanillaBreakBoundaryAndUnbreakable() {
        assertFalse(new ElytraDurability(true, 432, 431, false).usable());
        assertEquals(1, new ElytraDurability(true, 432, 430, false).remainingFlightSeconds());
        assertEquals(Double.POSITIVE_INFINITY, new ElytraDurability(true, 432, 432, true).remainingFlightSeconds());
        assertFalse(new ElytraDurability(true, 432, 0, false).canStart(Double.NaN));
    }
}
