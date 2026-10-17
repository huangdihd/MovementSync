package xin.bbtt.flight;

import xin.bbtt.physics.*;

/** Vanilla damage rules use the shared DamageModel through the physics adapter. */
public enum BuiltinDamageRule implements DamageRule {
    WALL, FALL;
    @Override public Damage damage(FlightContext c, FlightAction a, PhysicsState before, PhysicsResult after) {
        return this == WALL ? PublicFlightBridge.wall(c, a, before, after)
                : PublicFlightBridge.fall(c, a, before, after);
    }
}
