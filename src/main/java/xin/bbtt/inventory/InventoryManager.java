package xin.bbtt.inventory;

import xin.bbtt.physics.PlayerPose;
import xin.bbtt.MovementSync;

import lombok.Getter;
import lombok.Setter;
import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class InventoryManager {
    @Getter @Setter
    private int currentContainerId = 0;
    @Getter @Setter
    private int currentStateId = 0;
    @Getter @Setter
    private int heldSlot = 0;
    
    @Getter @Setter
    // Airborne inventory clicks raised no Grim or Vulcan alerts in live tests; "elytra status inventory-swap off" disables them.
    private volatile boolean airborneInventorySwap = true;
    @Getter
    private volatile boolean inventorySwapOpen;
    @Setter
    private volatile ItemStack carriedItem;
    private boolean inventoryRollback;

    private final Map<Integer, ItemStack[]> containers = new ConcurrentHashMap<>();

    /** Reads the authoritative chest-slot stack; never predicts or writes item damage. */
    public xin.bbtt.physics.ElytraDurability getElytraDurability() {
        ItemStack[] inventory = getInventory();
        ItemStack stack = inventory != null && inventory.length > 6 ? inventory[6] : null;
        return durability(stack);
    }

    private static xin.bbtt.physics.ElytraDurability durability(ItemStack stack) {
        ItemRegistry.ItemEntry item = stack == null ? null : ItemRegistry.Instance.getItem(stack.getId());
        if (stack == null || stack.getAmount() <= 0 || item == null || !"elytra".equals(item.getName())) {
            return new xin.bbtt.physics.ElytraDurability(false, 0, 0, false);
        }
        var patch = stack.getDataComponentsPatch();
        int max = xin.bbtt.physics.ElytraDurability.DEFAULT_MAX_DAMAGE;
        int damage = 0;
        boolean unbreakable = false;
        if (patch != null) {
            var types = patch.getDataComponents();
            var maxType = org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponentTypes.MAX_DAMAGE;
            var damageType = org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponentTypes.DAMAGE;
            var gliderType = org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponentTypes.GLIDER;
            var equipType = org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponentTypes.EQUIPPABLE;
            if (types.containsKey(gliderType) && patch.get(gliderType) == null
                    || types.containsKey(equipType) && (patch.get(equipType) == null
                    || patch.get(equipType).slot() != org.geysermc.mcprotocollib.protocol.data.game.entity.EquipmentSlot.CHESTPLATE)) {
                return new xin.bbtt.physics.ElytraDurability(false, 0, 0, false);
            }
            boolean nonDamageable = types.containsKey(maxType) && patch.get(maxType) == null
                    || types.containsKey(damageType) && patch.get(damageType) == null;
            max = patch.getOrDefault(maxType, max);
            damage = patch.getOrDefault(org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponentTypes.DAMAGE, 0);
            unbreakable = nonDamageable || patch.get(org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponentTypes.UNBREAKABLE) != null;
        }
        return new xin.bbtt.physics.ElytraDurability(true, Math.max(0, max), Math.max(0, damage), unbreakable);
    }

    /** Backpack reserves count only when airborne inventory interaction is enabled. */
    public synchronized double remainingFlightSeconds() {
        double seconds = getElytraDurability().remainingFlightSeconds();
        ItemStack[] items = getInventory();
        if (!canRemoveChestEquipment()) return seconds;
        if (items != null) for (int i = airborneInventorySwap ? 9 : 36; i < Math.min(45, items.length); i++) {
            if (canEquip(items[i])) seconds += durability(items[i]).remainingFlightSeconds();
        }
        return seconds;
    }

    public synchronized boolean canStartFlight(double distance) {
        return getElytraDurability().usable() && Double.isFinite(distance) && distance >= 0
                && remainingFlightSeconds() > xin.bbtt.physics.ElytraDurability.estimateFlightSeconds(distance);
    }

    /** Binding curse prevents survival-mode right-click replacement of worn equipment. */
    private boolean canRemoveChestEquipment() {
        ItemStack[] items = getInventory();
        ItemStack chest = items != null && items.length > 6 ? items[6] : null;
        if (chest == null || chest.getAmount() <= 0 || chest.getDataComponentsPatch() == null) return true;
        var enchantments = chest.getDataComponentsPatch().get(
                org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponentTypes.ENCHANTMENTS);
        Integer bindingId = xin.bbtt.listeners.RegistryDataListener.getEnchantmentIds().get("minecraft:binding_curse");
        return enchantments == null || bindingId == null || enchantments.getEnchantments().getOrDefault(bindingId, 0) <= 0;
    }

    private static boolean canEquip(ItemStack stack) {
        if (!durability(stack).usable()) return false;
        var patch = stack.getDataComponentsPatch();
        if (patch == null) return true;
        var type = org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponentTypes.EQUIPPABLE;
        if (!patch.getDataComponents().containsKey(type)) return true;
        var equipment = patch.get(type);
        return equipment != null && equipment.swappable()
                && equipment.slot() == org.geysermc.mcprotocollib.protocol.data.game.entity.EquipmentSlot.CHESTPLATE;
    }

    private int pendingSlot = -1;
    private ItemStack expectedStack;
    private int pendingOtherSlot = -1;
    private ItemStack expectedOtherStack;

    private boolean confirmed(ItemStack[] items) {
        return items != null && pendingSlot >= 0 && pendingSlot < items.length
                && java.util.Objects.equals(items[pendingSlot], expectedStack)
                && (pendingOtherSlot < 0 || pendingOtherSlot < items.length
                && java.util.Objects.equals(items[pendingOtherSlot], expectedOtherStack));
    }
    private int pendingTicks;
    private boolean swapFailed;
    public enum ElytraAction { READY, WAITING, LAND }

    /** Call once per flight tick, before rockets. No local inventory/damage prediction. */
    public synchronized ElytraAction tickElytraSwap(boolean onGround, java.util.function.Consumer<org.geysermc.mcprotocollib.network.packet.Packet> send,
                                                    java.util.function.IntSupplier sequence, float yaw, float pitch) {
        if (pendingSlot >= 0) {
            ItemStack[] items = getInventory();
            if (inventorySwapOpen && (inventoryRollback || carriedItem != null || currentContainerId != 0)) {
                closeInventorySwap(send);
                pendingSlot = -1;
                swapFailed = true;
                return ElytraAction.LAND;
            }
            if (confirmed(items)) {
                closeInventorySwap(send);
                MovementSync.getLogger().info("Elytra inventory update confirmed: remaining={}, seconds={}",
                        getElytraDurability().remainingDurability(), getElytraDurability().remainingFlightSeconds());
                pendingSlot = -1;
                expectedStack = null;
            } else if (++pendingTicks >= 40) {
                closeInventorySwap(send);
                pendingSlot = -1;
                expectedStack = null;
                swapFailed = true;
                return ElytraAction.LAND;
            } else return ElytraAction.WAITING;
        }
        var sync = MovementSync.INSTANCE;
        double height = 0;
        if (sync != null && !onGround) {
            var p = sync.position.get();
            var ground = sync.getWorld().findHighestCollisionTop(p.x - PlayerPose.HALF_WIDTH, xin.bbtt.world.World.getMinWorldY(),
                    p.z - PlayerPose.HALF_WIDTH, p.x + PlayerPose.HALF_WIDTH, p.y, p.z + PlayerPose.HALF_WIDTH);
            height = p.y - ground.orElse(xin.bbtt.world.World.getMinWorldY());
        }
        if (!getElytraDurability().shouldLand(height)) return ElytraAction.READY;
        if (swapFailed || currentContainerId != 0 || !canRemoveChestEquipment()) return ElytraAction.LAND;
        ItemStack[] items = getInventory();
        int best = -1;
        double budget = getElytraDurability().remainingFlightSeconds();
        if (items != null) for (int i = 36; i < Math.min(45, items.length); i++) {
            double candidate = durability(items[i]).remainingFlightSeconds();
            if (canEquip(items[i]) && candidate > budget) { best = i; budget = candidate; }
        }
        if (best < 0 && !onGround && airborneInventorySwap && carriedItem == null && items != null && items.length >= 45) {
            for (int i = 9; i < 36; i++) {
                double candidate = durability(items[i]).remainingFlightSeconds();
                if (canEquip(items[i]) && candidate > budget) { best = i; budget = candidate; }
            }
            if (best >= 0) {
                int destination = -1;
                for (int i = 36; i < 45; i++) if (items[i] == null || items[i].getAmount() <= 0) { destination = i; break; }
                if (destination < 0) destination = 36;
                pendingSlot = destination;
                expectedStack = items[best];
                pendingOtherSlot = best;
                expectedOtherStack = items[destination];
                pendingTicks = 0;
                inventoryRollback = false;
                inventorySwapOpen = true;
                MovementSync.getLogger().info("Airborne elytra inventory SWAP: slot={}, hotbar={}, state={}", best, destination - 36, currentStateId);
                // Omit prediction hashes to receive authoritative updates for both slots.
                // Matching hashes suppress confirmations in vanilla ScreenHandler.
                send.accept(new org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.inventory.ServerboundContainerClickPacket(
                        0, currentStateId, best,
                        org.geysermc.mcprotocollib.protocol.data.game.inventory.ContainerActionType.MOVE_TO_HOTBAR_SLOT,
                        org.geysermc.mcprotocollib.protocol.data.game.inventory.MoveToHotbarAction.values()[destination - 36],
                        null, java.util.Map.of()));
                return ElytraAction.WAITING;
            }
        }
        if (best < 0) return ElytraAction.LAND;
        MovementSync.getLogger().info("Swapping elytra from hotbar {}: remaining={}, replacement seconds={}",
                best - 36, getElytraDurability().remainingDurability(), budget);
        pendingSlot = 6;
        expectedStack = items[best];
        pendingOtherSlot = best;
        expectedOtherStack = items[6];
        pendingTicks = 0;
        int restore = heldSlot;
        send.accept(new org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundSetCarriedItemPacket(best - 36));
        send.accept(new org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket(
                org.geysermc.mcprotocollib.protocol.data.game.entity.player.Hand.MAIN_HAND, sequence.getAsInt(), yaw, pitch));
        send.accept(new org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundSetCarriedItemPacket(restore));
        return ElytraAction.WAITING;
    }

    private void closeInventorySwap(java.util.function.Consumer<org.geysermc.mcprotocollib.network.packet.Packet> send) {
        if (!inventorySwapOpen) return;
        send.accept(new org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.inventory.ServerboundContainerClosePacket(0));
        inventorySwapOpen = false;
        MovementSync.getLogger().info("Airborne elytra inventory closed: rollback={}", inventoryRollback);
    }

    /** Ground-only number-key transfer into an empty hotbar slot, awaiting server confirmation. */
    public synchronized ElytraAction prepareElytra(boolean onGround, java.util.function.Consumer<org.geysermc.mcprotocollib.network.packet.Packet> send) {
        if (!onGround || currentContainerId != 0) return ElytraAction.LAND;
        ItemStack[] items = getInventory();
        if (items == null || items.length < 45) return ElytraAction.READY;
        if (pendingSlot >= 0) {
            if (confirmed(items)) {
                MovementSync.getLogger().info("Elytra inventory update confirmed: remaining={}, seconds={}",
                        getElytraDurability().remainingDurability(), getElytraDurability().remainingFlightSeconds());
                pendingSlot = -1;
                expectedStack = null;
            } else if (++pendingTicks >= 40) {
                pendingSlot = -1;
                swapFailed = true;
                return ElytraAction.LAND;
            } else return ElytraAction.WAITING;
        }
        if (swapFailed) return ElytraAction.LAND;
        int empty = -1;
        for (int i = 36; i < 45; i++) if (items[i] == null || items[i].getAmount() <= 0) { empty = i; break; }
        if (empty < 0) return ElytraAction.READY;
        for (int i = 9; i < 36; i++) if (canEquip(items[i])) {
            pendingSlot = empty;
            expectedStack = items[i];
            pendingOtherSlot = i;
            expectedOtherStack = items[empty];
            pendingTicks = 0;
            send.accept(new org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.inventory.ServerboundContainerClickPacket(
                    0, currentStateId, i,
                    org.geysermc.mcprotocollib.protocol.data.game.inventory.ContainerActionType.MOVE_TO_HOTBAR_SLOT,
                    org.geysermc.mcprotocollib.protocol.data.game.inventory.MoveToHotbarAction.values()[empty - 36],
                    null, java.util.Map.of()));
            return ElytraAction.WAITING;
        }
        return ElytraAction.READY;
    }

    /** Call on disconnect/server change, before accepting the new session's inventory packets. */
    public synchronized void resetSession() {
        resetElytraSwap();
        containers.clear();
        currentContainerId = 0;
        currentStateId = 0;
        heldSlot = 0;
        carriedItem = null;
    }

    public synchronized void resetElytraSwap() {
        inventorySwapOpen = false;
        inventoryRollback = false;
        pendingSlot = -1;
        expectedStack = null;
        pendingTicks = 0;
        swapFailed = false;
        pendingOtherSlot = -1;
        expectedOtherStack = null;
    }

    public ItemStack getHeldItem() {
        ItemStack[] inventory = getInventory();
        if (inventory != null && inventory.length >= 45 && heldSlot >= 0 && heldSlot < 9) {
            return inventory[36 + heldSlot];
        }
        return null;
    }

    public int findBlock() {
        ItemStack[] inventory = getInventory();
        if (inventory == null || inventory.length < 45) return -1;
        // Search hotbar (36-44). An item id and a block-state id belong to
        // separate registries, so resolve the item to its name and look the
        // block up by name. Feeding the item id straight into isSolidBlock()
        // matches unrelated blocks and reports an empty hotbar (air, id 0) as
        // holding placeable blocks — which makes the planner build bridge/
        // pillar paths that can't actually be executed.
        for (int i = 36; i < 45; i++) {
            ItemStack stack = inventory[i];
            if (stack == null) continue;
            ItemRegistry.ItemEntry item = ItemRegistry.Instance.getItem(stack.getId());
            if (item == null) continue;
            if (xin.bbtt.Block.BlockStateParser.Instance.isSolidBlockByName(item.getName())) {
                return i - 36;
            }
        }
        return -1;
    }

    public int findBestTool(String material) {
        ItemStack[] inventory = getInventory();
        if (inventory == null) return -1;
        
        int bestSlot = -1;
        double maxMultiplier = 1.0;

        for (int i = 36; i < 45; i++) {
            if (inventory[i] == null) continue;
            double multiplier = ToolUtils.getToolMultiplier(inventory[i].getId(), material);
            if (multiplier > maxMultiplier) {
                maxMultiplier = multiplier;
                bestSlot = i - 36;
            }
        }
        return bestSlot;
    }

    public void switchToSlot(int slot) {
        if (slot < 0 || slot > 8) return;
        if (heldSlot == slot) return;
        xin.bbtt.mcbot.Bot.INSTANCE.getSession().send(new org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundSetCarriedItemPacket(slot));
        heldSlot = slot;
    }

    public synchronized void setContainerItems(int containerId, ItemStack[] items) {
        containers.put(containerId, items);
        if (containerId == 0 && inventorySwapOpen && !confirmed(items)) inventoryRollback = true;
    }

    public synchronized void setSlot(int containerId, int slot, ItemStack item) {
        // SetPlayerInventory-style updates use inventory indices rather than menu slots.
        if (containerId == -2) {
            containerId = 0;
            slot = slot < 9 ? slot + 36 : slot < 36 ? slot : slot < 40 ? 44 - slot : slot == 40 ? 45 : -1;
        }
        ItemStack[] items = containers.get(containerId);
        if (items != null && slot >= 0 && slot < items.length) {
            if (containerId == 0 && inventorySwapOpen
                    && (slot == pendingSlot && !java.util.Objects.equals(item, expectedStack)
                    || slot == pendingOtherSlot && !java.util.Objects.equals(item, expectedOtherStack))) {
                inventoryRollback = true;
            }
            items = items.clone();
            items[slot] = item;
            containers.put(containerId, items);
        }
    }

    /** One FIREWORKS flight component per hotbar rocket, in slot order (36..44).
     * Missing patches inherit the ordinary item default of flight=1; explicit removal uses 0.
     * Backpack and offhand rockets are excluded, matching MovementSync.countHotbarFireworks().
     */
    public int[] getHotbarFireworkFlightDurations() {
        ItemStack[] inventory = getInventory();
        if (inventory == null) return new int[0];
        var flights = java.util.stream.IntStream.builder();
        for (int slot = 36; slot < Math.min(45, inventory.length); slot++) {
            ItemStack stack = inventory[slot];
            if (stack == null || stack.getAmount() <= 0) continue;
            ItemRegistry.ItemEntry item = ItemRegistry.Instance.getItem(stack.getId());
            if (item == null || !"firework_rocket".equals(item.getName())) continue;
            int flight = 1;
            var patch = stack.getDataComponentsPatch();
            var type = org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponentTypes.FIREWORKS;
            if (patch != null && patch.getDataComponents().containsKey(type)) {
                var fireworks = patch.get(type);
                flight = fireworks == null ? 0 : fireworks.getFlightDuration();
            }
            for (int count = 0; count < stack.getAmount(); count++) flights.add(flight);
        }
        return flights.build().toArray();
    }

    public ItemStack[] getInventory() {
        return containers.get(0);
    }

    public ItemStack[] getOpenContainer() {
        return containers.get(currentContainerId);
    }

    public ItemStack[] getContainer(int id) {
        return containers.get(id);
    }
}
