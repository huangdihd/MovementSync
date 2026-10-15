package xin.bbtt.physics;

/** Conservative flight budget: vanilla attempts one damage every 20 glide ticks.
 * Unbreaking is deliberately not credited; its benefit is probabilistic.
 */
public record ElytraDurability(boolean equipped, int maxDamage, int damage, boolean unbreakable) {
    public static final int DEFAULT_MAX_DAMAGE = 432;
    public static final int LANDING_RESERVE_SECONDS = 10;
    public static final double ESTIMATED_BLOCKS_PER_SECOND = 20;

    public int remainingDurability() {
        return equipped ? (int) Math.max(0L, (long) maxDamage - Math.max(0, damage)) : 0;
    }

    public boolean usable() {
        return equipped && (unbreakable || remainingDurability() > 1);
    }

    /** Excludes the final point, at which vanilla refuses gliding. Not a guarantee of airtime. */
    public double remainingFlightSeconds() {
        return !usable() ? 0 : unbreakable ? Double.POSITIVE_INFINITY : remainingDurability() - 1;
    }

    public static double estimateFlightSeconds(double horizontalDistance) {
        if (!Double.isFinite(horizontalDistance) || horizontalDistance < 0) return Double.POSITIVE_INFINITY;
        return Math.ceil(horizontalDistance / ESTIMATED_BLOCKS_PER_SECOND) + LANDING_RESERVE_SECONDS;
    }

    public boolean canStart(double horizontalDistance) {
        return usable() && Double.isFinite(horizontalDistance) && horizontalDistance >= 0
                && remainingFlightSeconds() > estimateFlightSeconds(horizontalDistance);
    }

    public static double landingSeconds(double height) {
        return Math.max(0, height) / 12.0 + LANDING_RESERVE_SECONDS;
    }

    public boolean shouldLand(double height) {
        return remainingFlightSeconds() <= landingSeconds(height);
    }

    public boolean shouldLand() {
        return shouldLand(0);
    }
}
