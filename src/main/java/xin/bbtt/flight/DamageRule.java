package xin.bbtt.flight;

import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PhysicsResult;

/** Damage at a predicted tick; raw damage remains damaging even when fully mitigated. */
@FunctionalInterface
public interface DamageRule {
    Damage damage(FlightContext context, FlightAction action, PhysicsState before, PhysicsResult after);
    record Damage(double raw, double reduced) {
        public Damage {
            if (!Double.isFinite(raw) || !Double.isFinite(reduced) || raw < 0 || reduced < 0)
                throw new IllegalArgumentException("Invalid damage");
        }
    }
}
