package xin.bbtt.physics;

import org.geysermc.mcprotocollib.protocol.data.game.entity.Effect;
import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponentTypes;
import xin.bbtt.inventory.InventoryManager;
import xin.bbtt.listeners.RegistryDataListener;

import java.util.Map;

/** Known vanilla mitigation only; missing equipment, registry entries and effects grant no reduction. */
public record DamageParameters(int protection, int featherFalling, int resistanceAmplifier) {
    public static final DamageParameters NONE = new DamageParameters(0, 0, -1);

    public DamageParameters {
        protection = Math.max(0, Math.min(20, protection));
        featherFalling = Math.max(0, Math.min(20, featherFalling));
        resistanceAmplifier = Math.max(-1, Math.min(255, resistanceAmplifier));
    }

    /**
     * Incremental integration entry: pass the authoritative self-effect map (not movement flags).
     * MovementParameterState currently keeps that map private; unknown effects may be passed as null.
     */
    public static DamageParameters fromPlayer(InventoryManager inventory, Map<Effect, Integer> effects) {
        return fromInventory(inventory == null ? null : inventory.getInventory(),
                RegistryDataListener.getEnchantmentIds(), effects);
    }

    public static DamageParameters fromInventory(ItemStack[] inventory, Map<String, Integer> enchantmentIds,
            Map<Effect, Integer> effects) {
        int protection = 0;
        int featherFalling = 0;
        // Player container armor slots: head, chest, legs, feet. Hands/backpack do not protect.
        if (inventory != null) for (int slot = 5; slot <= 8 && slot < inventory.length; slot++) {
            ItemStack stack = inventory[slot];
            if (stack == null || stack.getAmount() <= 0 || stack.getDataComponentsPatch() == null) continue;
            var enchantments = stack.getDataComponentsPatch().get(DataComponentTypes.ENCHANTMENTS);
            if (enchantments == null) continue;
            protection = Math.min(20, protection + level(enchantments.getEnchantments(), enchantmentIds, "minecraft:protection"));
            // Vanilla's Feather Falling effect uses the ARMOR slot group, not only feet.
            featherFalling = Math.min(20, featherFalling + level(enchantments.getEnchantments(), enchantmentIds, "minecraft:feather_falling"));
        }
        Integer resistance = effects == null ? null : effects.get(Effect.RESISTANCE);
        return new DamageParameters(protection, featherFalling, resistance == null ? -1 : resistance);
    }

    private static int level(Map<Integer, Integer> levels, Map<String, Integer> ids, String name) {
        Integer id = ids == null ? null : ids.get(name);
        Integer level = id == null ? null : levels.get(id);
        return level == null ? 0 : Math.max(0, Math.min(20, level));
    }
}
