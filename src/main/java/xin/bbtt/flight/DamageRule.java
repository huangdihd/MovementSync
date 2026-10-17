package xin.bbtt.flight;

import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PhysicsResult;

/**
 * Calculates damage for one predicted tick, including the terminating tick.
 * Rules are additive and may run repeatedly during selection on the caller's thread;
 * implementations must be thread-safe, bounded, and free of live-player side effects.
 * Raw damage remains damaging even when fully mitigated.
 */
@FunctionalInterface
public interface DamageRule {
    /**
     * Evaluates one transition without applying damage to the player.
     *
     * @param context non-null planning snapshot
     * @param action non-null controls for this prediction (recovery controls when supplied)
     * @param before non-null state immediately before the tick
     * @param after non-null result of that tick
     * @return non-null damage contribution; use zero values for no damage
     */
    Damage damage(FlightContext context, FlightAction action, PhysicsState before, PhysicsResult after);
    /**
     * Validated damage contribution in health points, not hearts.
     * Component accessors return the supplied values without side effects.
     *
     * @param raw finite non-negative damage before mitigation
     * @param reduced finite non-negative damage after mitigation
     */
    record Damage(double raw, double reduced) {
        /**
         * Validates a damage contribution; no player state is changed.
         * @param raw damage before mitigation
         * @param reduced damage after mitigation
         * @throws IllegalArgumentException if either value is negative or nonfinite
         */
        public Damage {
            if (!Double.isFinite(raw) || !Double.isFinite(reduced) || raw < 0 || reduced < 0)
                throw new IllegalArgumentException("Invalid damage");
        }
    }
}
