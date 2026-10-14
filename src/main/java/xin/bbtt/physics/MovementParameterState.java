package xin.bbtt.physics;

import org.geysermc.mcprotocollib.protocol.data.game.entity.Effect;
import org.geysermc.mcprotocollib.protocol.data.game.entity.attribute.Attribute;
import org.geysermc.mcprotocollib.protocol.data.game.entity.attribute.AttributeType;
import org.geysermc.mcprotocollib.protocol.data.game.entity.attribute.ModifierOperation;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;

/** Packet-thread state; snapshots are published atomically and never expose mutable protocol objects. */
public final class MovementParameterState {
    private record Modifier(String identifier, double amount, ModifierOperation operation) {}
    private record AttributeValue(double base, List<Modifier> modifiers) {}

    private final Map<AttributeType.Builtin, AttributeValue> attributes = new EnumMap<>(AttributeType.Builtin.class);
    private final Map<Effect, Integer> effects = new EnumMap<>(Effect.class);
    private int food = 20;
    private boolean canFly;
    private volatile MovementParameters snapshot = MovementParameters.DEFAULT;

    public MovementParameters snapshot() {
        return snapshot;
    }

    public synchronized void reset() {
        attributes.clear();
        effects.clear();
        food = 20;
        canFly = false;
        publish();
    }

    /** Vanilla always copies bases on respawn, but only copies modifiers when requested. */
    public synchronized void respawn(boolean keepAttributeModifiers) {
        if (!keepAttributeModifiers) {
            attributes.replaceAll((type, value) -> new AttributeValue(value.base(), List.of()));
        }
        effects.clear();
        food = 20;
        publish();
    }

    public synchronized void updateAttributes(List<Attribute> updates) {
        for (Attribute attribute : updates) {
            if (!(attribute.getType() instanceof AttributeType.Builtin type)) continue;
            List<Modifier> modifiers = attribute.getModifiers().stream()
                    .map(modifier -> new Modifier(modifier.getId().asString(), modifier.getAmount(), modifier.getOperation()))
                    .toList();
            attributes.put(type, new AttributeValue(attribute.getValue(), modifiers));
        }
        publish();
    }

    /** Client effects remain present until the authoritative remove packet, even at duration zero. */
    public synchronized void updateEffect(Effect effect, int amplifier) {
        effects.put(effect, Math.max(0, Math.min(255, amplifier)));
        publish();
    }

    public synchronized void removeEffect(Effect effect) {
        effects.remove(effect);
        publish();
    }

    public synchronized void updateFood(int food) {
        this.food = food;
        publish();
    }

    public synchronized void updateAbilities(boolean canFly) {
        this.canFly = canFly;
        publish();
    }

    private void publish() {
        int jumpBoost = effects.getOrDefault(Effect.JUMP_BOOST, -1);
        snapshot = new MovementParameters(
                value(AttributeType.Builtin.MOVEMENT_SPEED, false),
                value(AttributeType.Builtin.MOVEMENT_SPEED, true),
                value(AttributeType.Builtin.JUMP_STRENGTH, false),
                jumpBoost < 0 ? 0 : 0.1F * (jumpBoost + 1.0F),
                value(AttributeType.Builtin.GRAVITY, false),
                value(AttributeType.Builtin.STEP_HEIGHT, false),
                value(AttributeType.Builtin.SNEAKING_SPEED, false),
                effects.containsKey(Effect.SLOW_FALLING),
                effects.getOrDefault(Effect.LEVITATION, -1),
                (food > 6 || canFly) && !effects.containsKey(Effect.BLINDNESS));
    }

    private double value(AttributeType.Builtin type, boolean sprinting) {
        double defaultBase = switch (type) {
            case MOVEMENT_SPEED -> (double) 0.1F;
            case JUMP_STRENGTH -> (double) 0.42F;
            default -> type.getDef();
        };
        AttributeValue attribute = attributes.getOrDefault(type, new AttributeValue(defaultBase, List.of()));
        List<Modifier> modifiers = attribute.modifiers();
        double additive = attribute.base();
        for (Modifier modifier : modifiers) {
            if (isSprintModifier(type, modifier)) continue;
            if (modifier.operation() == ModifierOperation.ADD) additive += modifier.amount();
        }
        double result = additive;
        for (Modifier modifier : modifiers) {
            if (isSprintModifier(type, modifier)) continue;
            if (modifier.operation() == ModifierOperation.ADD_MULTIPLIED_BASE) result += additive * modifier.amount();
        }
        for (Modifier modifier : modifiers) {
            if (isSprintModifier(type, modifier)) continue;
            if (modifier.operation() == ModifierOperation.ADD_MULTIPLIED_TOTAL) result *= 1 + modifier.amount();
        }
        if (type == AttributeType.Builtin.MOVEMENT_SPEED && sprinting) result *= 1.0 + (double) 0.3F;
        return Double.isNaN(result) ? type.getMin() : Math.max(type.getMin(), Math.min(type.getMax(), result));
    }

    private static boolean isSprintModifier(AttributeType.Builtin type, Modifier modifier) {
        return type == AttributeType.Builtin.MOVEMENT_SPEED && modifier.identifier().equals("minecraft:sprinting");
    }
}
