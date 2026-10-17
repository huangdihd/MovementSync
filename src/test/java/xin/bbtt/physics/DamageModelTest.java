package xin.bbtt.physics;

import org.geysermc.mcprotocollib.protocol.data.game.entity.Effect;
import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponentTypes;
import org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponents;
import org.geysermc.mcprotocollib.protocol.data.game.item.component.ItemEnchantments;
import org.junit.jupiter.api.Test;

import java.util.HashMap;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

class DamageModelTest {
    @Test void fallUsesFloorEpsilonAndAttributesWithoutChangingLegacyApi() {
        assertEquals(0, DamageModel.rawFall(3.99, 1, MovementParameters.DEFAULT));
        assertEquals(1, DamageModel.rawFall(3.9999995, 1, MovementParameters.DEFAULT));
        assertEquals(17, DamageModel.rawFall(20, 1, MovementParameters.DEFAULT));
        assertEquals(3, DamageModel.rawFall(20, (double) 0.2F, MovementParameters.DEFAULT));
        assertEquals(0, DamageModel.rawFall(20, 0, MovementParameters.DEFAULT));
        var attributes = new MovementParameters(.1, .13, .42, 0, .08, .6, .3,
                false, -1, true, 5, 2);
        assertEquals(6, DamageModel.rawFall(8, 1, attributes));
        for (double distance : new double[]{0, 3, 3.99, 4, 20, 100}) {
            assertEquals(FallPrediction.damage(distance, 1, attributes), DamageModel.rawFall(distance, 1, attributes));
        }
    }

    @Test void authoritativeEffectsAndHealthSnapshotsTrackRemovalAndRespawn() {
        var state = new MovementParameterState();
        state.updateHealth(7);
        state.updateEffect(Effect.RESISTANCE, 1);
        var snapshot = state.effectsSnapshot();
        assertEquals(7, state.health());
        assertEquals(1, snapshot.get(Effect.RESISTANCE));
        state.removeEffect(Effect.RESISTANCE);
        assertTrue(state.effectsSnapshot().isEmpty());
        assertEquals(1, snapshot.get(Effect.RESISTANCE));
        state.respawn(false);
        assertEquals(20, state.health());
        assertTrue(state.effectsSnapshot().isEmpty());
    }

    @Test void wallUsesCollisionGateAndFloatDamageNotIntegerRounding() {
        assertTrue(DamageModel.wall(false, 2, 0, null).harmless());
        assertTrue(DamageModel.wall(true, .3, 0, null).harmless());
        assertTrue(DamageModel.wall(true, .2, .4, null).harmless());
        assertEquals(1.5F, DamageModel.wall(true, .45, 0, null).raw());
        assertEquals(7, DamageModel.wall(true, 1.5, .5, null).raw());
    }

    @Test void resistancePrecedesCappedProtectionAndFeatherFallingOnlyProtectsFalls() {
        var parameters = new DamageParameters(4, 4, 0);
        assertEquals(8 * (1 - 16 / 25.0F), DamageModel.apply(10, DamageModel.Source.FALL, parameters).reduced());
        assertEquals(8 * (1 - 4 / 25.0F), DamageModel.apply(10, DamageModel.Source.WALL, parameters).reduced());
        assertEquals(10 * (1 - 20 / 25.0F), DamageModel.apply(10, DamageModel.Source.FALL,
                new DamageParameters(20, 4, -1)).reduced());
        assertEquals(10, DamageModel.apply(10, DamageModel.Source.FALL, null).reduced());
    }

    @Test void fullMitigationDoesNotQualifyAsHarmlessAndLethalityIncludesAbsorption() {
        var immune = DamageModel.apply(10, DamageModel.Source.FALL, new DamageParameters(0, 0, 4));
        assertEquals(0, immune.reduced());
        assertFalse(immune.harmless());
        assertFalse(immune.lethal(1, 0));
        var hit = DamageModel.apply(10, DamageModel.Source.WALL, null);
        assertTrue(hit.lethal(8, 2));
        assertFalse(hit.lethal(8, 3));
        assertTrue(DamageModel.fall(3, 1, MovementParameters.DEFAULT, null).harmless());
    }

    @Test void actualEquipmentComponentsAndEffectsAreReadWithoutBackpackOrHandProtection() {
        var patch = new DataComponents(new HashMap<>());
        patch.put(DataComponentTypes.ENCHANTMENTS, new ItemEnchantments(Map.of(123, 4, 456, 4)));
        var armor = new ItemStack(1, 1, patch);
        var inventory = new ItemStack[46];
        inventory[5] = armor;
        inventory[8] = armor;
        inventory[9] = armor;
        inventory[45] = armor;
        var ids = Map.of("minecraft:protection", 123, "minecraft:feather_falling", 456);
        assertEquals(new DamageParameters(8, 8, 1), DamageParameters.fromInventory(inventory, ids, Map.of(Effect.RESISTANCE, 1)));
        assertEquals(DamageParameters.NONE, DamageParameters.fromInventory(inventory, Map.of(), null));
        assertEquals(DamageParameters.NONE, DamageParameters.fromPlayer(null, null));
        assertEquals(DamageParameters.NONE, DamageParameters.fromInventory(null, ids, Map.of(Effect.SLOW_FALLING, 0)));
    }
}
