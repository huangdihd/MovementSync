package xin.bbtt.recovery;

import org.joml.Vector3i;
import java.time.Duration;
import java.util.function.LongSupplier;

/** In-memory intent, deliberately separate from session-bound movements and paths. */
public final class StaticGotoRecovery {
    public static final long MAX_AGE_MILLIS = Duration.ofMinutes(30).toMillis();
    private final LongSupplier clock;
    private Task task;
    private boolean pending;
    private boolean teleported;
    private long suspendedAt;
    private int quietTicks;
    public static final int STABLE_TICKS = 20;

    public StaticGotoRecovery() { this(System::currentTimeMillis); }
    public StaticGotoRecovery(LongSupplier clock) { this.clock = clock; }

    public record Task(int x, int y, int z, boolean walk, String dimension, long createdAt) {
        public Vector3i goal() { return new Vector3i(x, y, z); }
    }

    public synchronized void remember(Vector3i goal, boolean walk, String dimension) {
        task = new Task(goal.x, goal.y, goal.z, walk, dimension, clock.getAsLong());
        pending = false;
    }

    public synchronized Task getTask() { return task; }
    public synchronized boolean isPending() { return pending; }
    public synchronized void clear() { task = null; pending = false; }
    public synchronized void suspend() {
        if (!pending) suspendedAt = clock.getAsLong();
        pending = task != null;
        teleported = false;
        quietTicks = 0;
    }
    public synchronized void onFirstTeleport() { onTeleport(); }
    public synchronized boolean onTeleport() {
        boolean first = !teleported;
        teleported = true;
        quietTicks = 0;
        return first;
    }
    public synchronized void tick() { if (teleported && quietTicks < STABLE_TICKS) quietTicks++; }
    public synchronized boolean hasPosition() { return teleported; }

    public synchronized boolean expire() {
        if (!pending || task == null || clock.getAsLong() - suspendedAt < MAX_AGE_MILLIS) return false;
        clear();
        return true;
    }

    /** Claim once per suspension; callers must not refresh the original creation time. */
    public synchronized Task claim(boolean game, String dimension) {
        expire();
        if (!pending || !game || !teleported || quietTicks < STABLE_TICKS || task == null || dimension == null) return null;
        if (task.dimension() == null || !task.dimension().equals(dimension)) {
            xin.bbtt.MovementSync.getLogger().info("Static goto abandoned: dimension changed from {} to {}", task.dimension(), dimension);
            clear();
            return null;
        }
        pending = false;
        return task;
    }
}
