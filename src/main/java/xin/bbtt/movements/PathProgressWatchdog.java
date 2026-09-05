package xin.bbtt.movements;

/** Counts PathMovement ticks since the last completed path node. */
final class PathProgressWatchdog {
    private final int maximumTicksWithoutProgress;
    private int ticksWithoutProgress;

    PathProgressWatchdog(int maximumTicksWithoutProgress) {
        if (maximumTicksWithoutProgress < 1) {
            throw new IllegalArgumentException("maximumTicksWithoutProgress must be positive");
        }
        this.maximumTicksWithoutProgress = maximumTicksWithoutProgress;
    }

    boolean tick() {
        return ++ticksWithoutProgress >= maximumTicksWithoutProgress;
    }

    void nodeAdvanced() {
        ticksWithoutProgress = 0;
    }
}
