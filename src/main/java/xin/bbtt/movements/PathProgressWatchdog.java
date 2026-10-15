package xin.bbtt.movements;

import java.util.function.LongSupplier;
import xin.bbtt.pathfinding.Node;

/** Measures execution opportunities, not planner time or scheduler catch-up ticks. */
final class PathProgressWatchdog {
    private static final long TICK_NANOS = 50_000_000L;
    private final long maximumNanosWithoutProgress;
    private final LongSupplier nanoTime;
    private long nanosWithoutProgress;
    private long previousSample;
    private boolean sampling;
    private Node lastCompletedNode;

    PathProgressWatchdog(int maximumTicksWithoutProgress) {
        this(maximumTicksWithoutProgress, System::nanoTime);
    }

    PathProgressWatchdog(int maximumTicksWithoutProgress, LongSupplier nanoTime) {
        if (maximumTicksWithoutProgress < 1) {
            throw new IllegalArgumentException("maximumTicksWithoutProgress must be positive");
        }
        this.maximumNanosWithoutProgress = maximumTicksWithoutProgress * TICK_NANOS;
        this.nanoTime = nanoTime;
    }

    boolean tick() {
        long now = nanoTime.getAsLong();
        if (sampling) {
            // A delayed callback is at most one movement opportunity; back-to-back
            // fixed-rate callbacks must not manufacture seconds of stalled motion.
            nanosWithoutProgress += Math.min(TICK_NANOS, Math.max(0L, now - previousSample));
        }
        previousSample = now;
        sampling = true;
        return nanosWithoutProgress >= maximumNanosWithoutProgress;
    }

    void nodeAdvanced(Node node) {
        // Replanning can expose the same start node again without physical progress.
        if (node.equals(lastCompletedNode)) return;
        lastCompletedNode = node;
        nanosWithoutProgress = 0;
    }

    void excludeElapsedTime() {
        previousSample = nanoTime.getAsLong();
    }

    void pause() {
        sampling = false;
    }
}
