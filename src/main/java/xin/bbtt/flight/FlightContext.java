package xin.bbtt.flight;

import java.util.Map;
import java.util.Objects;
import org.joml.Vector3d;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

/** Read-only tick inputs. World is a query view by convention; hooks must not mutate it. */
public record FlightContext(World world, PhysicsState state, MovementParameters movement,
        DamageParameters damage, double health, Vector3d target, float yaw, float pitch,
        int fireworks, boolean attached, double radius, boolean emergency,
        ElytraFlightController.Decision legacyDecision, Map<String, Integer> inventory) {
    public FlightContext {
        Objects.requireNonNull(world); Objects.requireNonNull(state);
        Objects.requireNonNull(movement); Objects.requireNonNull(damage);
        target = new Vector3d(Objects.requireNonNull(target));
        inventory = Map.copyOf(inventory);
        if (!state.position().isFinite() || !state.velocity().isFinite() || !target.isFinite()
                || !Float.isFinite(yaw) || !Float.isFinite(pitch) || !Double.isFinite(health)
                || health < 0 || !Double.isFinite(radius) || radius <= 0 || fireworks < 0)
            throw new IllegalArgumentException("Invalid flight inputs");
    }
    @Override public Vector3d target() { return new Vector3d(target); }
    public FlightPhase phase() {
        return legacyDecision == null ? (emergency ? FlightPhase.Standard.FORCED_LANDING : FlightPhase.Standard.CRUISE)
                : FlightPhase.Standard.valueOf(legacyDecision.phase().name());
    }
}
