package xin.bbtt.physics;

/** Vanilla 1.21.11 fall and gliding-wall damage; both sources bypass armor/toughness. */
final class DamageModel {
    private DamageModel() {}

    enum Source { FALL, WALL }

    record Damage(float raw, float reduced) {
        /** Mitigation must never turn an intrinsically damaging route into a no-damage route. */
        boolean harmless() { return raw == 0; }

        /** Absorption is a separate health pool, not a reduction in incoming damage. */
        boolean lethal(double health, double absorption) {
            return reduced > 0 && reduced >= health + Math.max(0, absorption);
        }
    }

    static int rawFall(double distance, double blockMultiplier, MovementParameters parameters) {
        return Math.max(0, (int) Math.floor((distance + 1.0e-6 - parameters.safeFallDistance())
                * blockMultiplier * parameters.fallDamageMultiplier()));
    }

    static Damage fall(double distance, double blockMultiplier, MovementParameters movement,
            DamageParameters mitigation) {
        return apply(rawFall(distance, blockMultiplier, movement), Source.FALL, mitigation);
    }

    /**
     * Speeds are horizontal magnitudes: before calcGlidingVelocity (after tick snapping),
     * and after Entity.move clips velocity, BEFORE a firework's later entity tick boost.
     * Do not substitute displacement or the post-firework PhysicsResult velocity.
     */
    static Damage wall(boolean horizontalCollision, double oldSpeed, double clippedSpeed,
            DamageParameters mitigation) {
        float raw = horizontalCollision ? Math.max(0, (float) ((oldSpeed - clippedSpeed) * 10 - 3)) : 0;
        return apply(raw, Source.WALL, mitigation);
    }

    static Damage apply(float raw, Source source, DamageParameters mitigation) {
        raw = Math.max(0, raw);
        if (mitigation == null) mitigation = DamageParameters.NONE;
        float reduced = raw;
        if (mitigation.resistanceAmplifier() >= 0) {
            reduced = Math.max(reduced * (25 - (mitigation.resistanceAmplifier() + 1) * 5) / 25.0F, 0);
        }
        int protection = Math.min(20, mitigation.protection()
                + (source == Source.FALL ? mitigation.featherFalling() * 3 : 0));
        reduced *= 1.0F - protection / 25.0F;
        return new Damage(raw, reduced);
    }
}
