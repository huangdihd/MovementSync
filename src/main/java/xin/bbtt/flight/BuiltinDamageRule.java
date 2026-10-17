package xin.bbtt.flight;

import xin.bbtt.physics.*;

/**
 * Stateless, thread-safe vanilla damage rules using the shared damage model.
 * Evaluation is synchronous and reads prediction/world data without damaging the
 * live player. The physics adapter is an internal implementation detail, not an
 * extension entry point.
 */
public enum BuiltinDamageRule implements DamageRule {
    /** Horizontal impact damage, with vanilla glide/firework impact handling. */
    WALL,
    /** Ground-impact damage including landing-block and movement-effect mitigation. */
    FALL,
    /** Conservative swept lava/lava-cauldron contact estimate, independent of flight mode. */
    LAVA,
    /** Conservative swept fire, lit campfire and hot magma contact estimate. */
    FIRE;
    /**
     * Evaluates this rule for one predicted transition, including a terminating tick.
     * @param c non-null planning snapshot
     * @param a non-null controls used for this prediction
     * @param before non-null state before the tick
     * @param after non-null tick result
     * @return non-null raw and mitigated damage contribution in health points
     */
    @Override public Damage damage(FlightContext c, FlightAction a, PhysicsState before, PhysicsResult after) {
        return switch (this) {
            case WALL -> PublicFlightBridge.wall(c, a, before, after);
            case FALL -> PublicFlightBridge.fall(c, a, before, after);
            case LAVA -> PublicFlightBridge.hazard(c, before, after, true);
            case FIRE -> PublicFlightBridge.hazard(c, before, after, false);
        };
    }
}
