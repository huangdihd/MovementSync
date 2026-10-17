package xin.bbtt.physics;

import xin.bbtt.physics.FlightSafety.Impact;
import xin.bbtt.world.World;

/** Vanilla pull-up trajectories; safety owns their damage classification. */
final class VanillaGlideRecovery implements FlightSafety.RecoveryStrategy {
    // Three seconds covers transient pull-up response without requiring a complete
    // landing. Exhaustion is UNKNOWN, never a fabricated numeric damage penalty.
    private static final int RECOVERY_TICKS = 60;
    private final FlightSafety safety;
    private final float[] pitches;

    VanillaGlideRecovery(FlightSafety safety, float[] pitches) {
        this.safety = safety;
        this.pitches = pitches.clone();
    }

    @Override public Impact recover(World world, PhysicsState initial, float yaw, boolean attached) {
        Impact best = Impact.UNKNOWN;
        for (float pitch : pitches) {
            FlightSafety.CheckedTrajectory checked = safety.simulate(world, initial,
                    safety.stepper(yaw, pitch, attached), RECOVERY_TICKS, yaw, pitch, attached, true);
            Impact impact = checked.terminalImpact();
            if (impact == Impact.SAFE) return Impact.SAFE;
            if (impact != null && safety.compareImpact(impact, best) < 0) best = impact;
        }
        return best;
    }
}
