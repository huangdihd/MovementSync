package xin.bbtt.flight;

import java.util.Map;
import java.util.Objects;
import org.joml.Vector3d;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

/**
 * Read-only inputs shared by all modes during one decision or transition preview.
 * Prediction hooks may query these inputs but must not mutate the world or live physics.
 * The world is a live query view, not a thread-isolated snapshot; callers must use the
 * navigation decision thread. Inventory and target are defensively copied.
 *
 * @param world non-null collision/environment query view
 * @param state common initial physics state for every candidate
 * @param movement movement parameters used by prediction and execution
 * @param damage current vanilla mitigation parameters
 * @param health current health, used to classify lethal damage
 * @param target current target aim point; accessor returns a copy
 * @param yaw current yaw in degrees
 * @param pitch current pitch in degrees
 * @param fireworks available rocket budget after lifecycle cooldown restrictions
 * @param attached whether rocket propulsion is already active
 * @param radius positive arrival radius in blocks
 * @param emergency whether damage-first emergency control is requested
 * @param inventory non-null immutable item-count snapshot
 */
public record FlightContext(World world, PhysicsState state, MovementParameters movement,
        DamageParameters damage, double health, Vector3d target, float yaw, float pitch,
        int fireworks, boolean attached, double radius, boolean emergency,
        Map<String, Integer> inventory) {
    /** Creates validated inputs without changing live state; null inputs are rejected. */
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
    /** Returns a non-null independent target vector; mutating it has no live effect. */
    @Override public Vector3d target() { return new Vector3d(target); }
    /** Returns the default phase for this request; modes may supply a more specific phase. */
    public FlightPhase phase() {
        return emergency ? FlightPhase.Standard.FORCED_LANDING : FlightPhase.Standard.CRUISE;
    }
}
