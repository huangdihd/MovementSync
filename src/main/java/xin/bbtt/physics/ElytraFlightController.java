package xin.bbtt.physics;

import org.joml.Vector3d;
import xin.bbtt.world.World;

/** Bounded trajectory steering. The caller owns rocket lifetime and flight lifecycle. */
public final class ElytraFlightController {
    private final FlightSelector selector;

    public ElytraFlightController() { selector = new FlightSelector(); }
    public ElytraFlightController(double gravity, boolean slowFalling) {
        selector = new FlightSelector(gravity, slowFalling);
    }
    /** Damage-aware emergency prediction; unknown absorption is conservatively ignored. */
    public ElytraFlightController(MovementParameters movement, DamageParameters mitigation, double health) {
        selector = new FlightSelector(movement, mitigation, health);
    }

    public enum Phase implements FlightPhase { CLIMB, CRUISE, DESCENT, AVOIDANCE, LANDING, FORCED_LANDING, LANDED }
    public record Decision(float yaw, float pitch, boolean firework, Phase phase) {}

    public Decision decide(World world, PhysicsState state, float yaw, float pitch, boolean attached, Vector3d target, int fireworks, double radius) {
        return selector.decide(world, state, yaw, pitch, attached, target, fireworks, radius);
    }

    public Decision decide(World world, PhysicsState state, float yaw, float pitch, boolean attached, FlightTarget target, int fireworks, double radius) {
        return selector.decide(world, state, yaw, pitch, attached, target, fireworks, radius);
    }

    /** Corridor-only candidate filter; rejected plugin-damage candidates do not hide safe alternatives. */
    public Decision decide(World world, PhysicsState state, float yaw, float pitch, boolean attached,
            WaypointFlightTarget target, int fireworks, double radius,
            java.util.function.Predicate<Decision> allowed) {
        return selector.decideLow(world, state, yaw, pitch, attached, target, fireworks, radius,
                java.util.Objects.requireNonNull(allowed));
    }

    /** Corridor steering with the caller's actual rocket inventory, cooldown and landing memory. */
    public Decision decide(World world, PhysicsState state, float yaw, float pitch, boolean attached,
            WaypointFlightTarget target, int fireworks, double radius,
            HazardRecovery.Context context, java.util.function.Predicate<Decision> allowed) {
        return selector.decideLow(world, state, yaw, pitch, attached, target, fireworks, radius,
                java.util.Objects.requireNonNull(context), java.util.Objects.requireNonNull(allowed));
    }

    public Decision forceLand(World world, PhysicsState state, float yaw, float pitch, boolean attached, int fireworks, double radius) {
        return selector.forceLand(world, state, yaw, pitch, attached, fireworks, radius);
    }

    public Decision forceLand(World world, PhysicsState state, float yaw, float pitch, boolean attached, int fireworks, double radius, xin.bbtt.movements.LandingMemory memory) {
        return selector.forceLand(world, state, yaw, pitch, attached, fireworks, radius, memory);
    }

    public Decision forceLand(World world, PhysicsState state, float yaw, float pitch, boolean attached, int[] flights, double radius, xin.bbtt.movements.LandingMemory memory, LandingSearch search) {
        return selector.forceLand(world, state, yaw, pitch, attached, flights, radius, memory, search);
    }

    public Decision emergencyHold(World world, PhysicsState state, float yaw, float pitch, boolean attached) {
        return selector.emergencyHold(world, state, yaw, pitch, attached);
    }

    public static Vector3d selectForcedLandingTarget(World world, PhysicsState state, int fireworks, boolean attached) {
        return VanillaGlideMode.selectForcedLandingTarget(world, state, fireworks, attached);
    }

    static Vector3d selectForcedLandingTarget(World world, PhysicsState state, int fireworks, boolean attached, MovementParameters parameters) {
        return VanillaGlideMode.selectForcedLandingTarget(world, state, fireworks, attached, parameters);
    }

    public static Vector3d landingSite(World world, double x, double z, double ceiling) {
        return VanillaGlideMode.landingSite(world, x, z, ceiling);
    }

    static float[] landingPitches() {
        return VanillaGlideMode.landingPitches();
    }

    // Retain the legacy reflection seam used by the eager-search reference benchmark.
    private Decision emergencyDecision(World world, PhysicsState state, float yaw, float pitch,
            boolean attached, Vector3d target, double radius, float heading,
            boolean needsBoost, boolean firework, boolean hold) {
        return selector.emergencyDecision(world, state, yaw, pitch, attached, target, radius,
                heading, needsBoost, firework, hold);
    }

    double emergencyDamage(World world, PhysicsState state, float yaw, float pitch, boolean attached) {
        return selector.emergencyDamage(world, state, yaw, pitch, attached);
    }
}
