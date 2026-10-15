package xin.bbtt.physics;

import net.kyori.adventure.key.Key;
import org.geysermc.mcprotocollib.protocol.data.game.entity.Effect;
import org.geysermc.mcprotocollib.protocol.data.game.entity.attribute.Attribute;
import org.geysermc.mcprotocollib.protocol.data.game.entity.attribute.AttributeModifier;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.geysermc.mcprotocollib.protocol.data.game.entity.attribute.AttributeType.Builtin.*;
import static org.geysermc.mcprotocollib.protocol.data.game.entity.attribute.ModifierOperation.*;
import static org.junit.jupiter.api.Assertions.*;

class MovementParametersTest {
    @Test
    void defaultsMatchPlayerRatherThanGenericMobAttributes() {
        MovementParameters parameters = new MovementParameterState().snapshot();
        assertEquals((double) 0.1F, parameters.movementSpeed());
        assertEquals((double) 0.42F, parameters.jumpVelocity(1));
        assertEquals(0.08, parameters.gravity());
        assertEquals(0.6, parameters.stepHeight());
        assertEquals(0.3, parameters.sneakingSpeed());
        assertEquals(-1, parameters.levitationAmplifier());
        assertTrue(parameters.canSprint());
    }

    @Test
    void evaluatesOperationsInVanillaOrderAndReplacesSprintModifierBeforeClamping() {
        MovementParameterState state = new MovementParameterState();
        state.updateAttributes(List.of(new Attribute(MOVEMENT_SPEED, 0.1, List.of(
                new AttributeModifier(Key.key("test:total"), 0.5, ADD_MULTIPLIED_TOTAL),
                new AttributeModifier(Key.key("minecraft:sprinting"), 0.3F, ADD_MULTIPLIED_TOTAL),
                new AttributeModifier(Key.key("test:base"), 0.2, ADD_MULTIPLIED_BASE),
                new AttributeModifier(Key.key("test:add"), 0.1, ADD)))));
        assertEquals(0.36, state.snapshot().movementSpeed(), 1e-12);
        assertEquals(0.36 * (1 + (double) 0.3F), state.snapshot().movementSpeed(true), 1e-12);
        state.updateAttributes(List.of(new Attribute(MOVEMENT_SPEED, 1000)));
        assertEquals(1024, state.snapshot().movementSpeed(true));
    }

    @Test
    void attributeReplacementIsPartialAndSnapshotsDoNotAliasPacketLists() {
        MovementParameterState state = new MovementParameterState();
        List<AttributeModifier> modifiers = new ArrayList<>();
        modifiers.add(new AttributeModifier(Key.key("test:add"), 0.2, ADD));
        state.updateAttributes(List.of(new Attribute(MOVEMENT_SPEED, 0.1, modifiers), new Attribute(STEP_HEIGHT, 1.2)));
        MovementParameters previous = state.snapshot();
        modifiers.clear();
        state.updateFood(10);
        assertEquals(0.3, state.snapshot().movementSpeed(), 1e-12);
        state.updateAttributes(List.of(new Attribute(MOVEMENT_SPEED, 0.2)));
        assertEquals(0.2, state.snapshot().movementSpeed());
        assertEquals(1.2, state.snapshot().stepHeight());
        assertEquals(0.3, previous.movementSpeed(), 1e-12);
    }

    @Test
    void speedAndSlownessPacketsDoNotDoubleApplyAttributeModifiers() {
        MovementParameterState state = new MovementParameterState();
        state.updateAttributes(List.of(new Attribute(MOVEMENT_SPEED, 0.1, List.of(
                new AttributeModifier(Key.key("minecraft:effect.speed"), 0.4, ADD_MULTIPLIED_TOTAL),
                new AttributeModifier(Key.key("minecraft:effect.slowness"), -0.15, ADD_MULTIPLIED_TOTAL)))));
        double speed = state.snapshot().movementSpeed();
        state.updateEffect(Effect.SPEED, 1);
        state.updateEffect(Effect.SLOWNESS, 0);
        assertEquals(speed, state.snapshot().movementSpeed());
        state.removeEffect(Effect.SPEED);
        assertEquals(speed, state.snapshot().movementSpeed());
    }

    @Test
    void effectsAndAttributesResolveJumpGravityAndLevitationWithoutDrag() {
        MovementParameterState state = new MovementParameterState();
        state.updateAttributes(List.of(new Attribute(JUMP_STRENGTH, 0.8), new Attribute(GRAVITY, 0.04)));
        state.updateEffect(Effect.JUMP_BOOST, 1);
        assertEquals((double) (0.8F * 0.5F + 0.1F * 2), state.snapshot().jumpVelocity(0.5));
        state.updateEffect(Effect.SLOW_FALLING, 0);
        assertEquals(0.04, state.snapshot().effectiveGravity(0.1));
        assertEquals(0.01, state.snapshot().effectiveGravity(0));
        assertEquals(-0.31, state.snapshot().verticalVelocityAfterForces(-0.3), 1e-12);
        state.updateEffect(Effect.LEVITATION, 1);
        assertEquals(-0.22, state.snapshot().verticalVelocityAfterForces(-0.3), 1e-12);
        state.removeEffect(Effect.LEVITATION);
        state.updateAttributes(List.of(new Attribute(GRAVITY, -0.1)));
        assertEquals(-0.1, state.snapshot().effectiveGravity(-0.3));
        state.removeEffect(Effect.JUMP_BOOST);
        assertEquals(0, state.snapshot().jumpBoostVelocity());
    }

    @Test
    void clampsAttributesLikeVanillaIncludingNanAndNegativeGravity() {
        MovementParameterState state = new MovementParameterState();
        state.updateAttributes(List.of(new Attribute(STEP_HEIGHT, 100), new Attribute(JUMP_STRENGTH, -1),
                new Attribute(GRAVITY, -2), new Attribute(MOVEMENT_SPEED, Double.NaN),
                new Attribute(SNEAKING_SPEED, Double.POSITIVE_INFINITY)));
        assertEquals(10, state.snapshot().stepHeight());
        assertEquals(0, state.snapshot().jumpStrength());
        assertEquals(-1, state.snapshot().gravity());
        assertEquals(0, state.snapshot().movementSpeed());
        assertEquals(1, state.snapshot().sneakingSpeed());
    }
}
