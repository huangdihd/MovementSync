package xin.bbtt.physics;

/** Shared minimum-lifetime resource clock. One call consumes one simulated travel tick. */
public final class FireworkSchedule {
    /** Tick-boundary resource state, safe to retain alongside a physical witness. */
    public record Checkpoint(long boostTicks, int cooldownTicks, int consumed,
                             int pendingTicks, long pendingBoostTicks) {}

    /** A request is distinct from acknowledged thrust. checkpoint is the end of this tick. */
    public record Tick(boolean useRequested, boolean powered, Checkpoint checkpoint) {}

    public Checkpoint checkpoint() {
        return new Checkpoint(boost, cooldown, used, pending, pendingBoost);
    }
    private long boost;
    private final int[] flights;
    private final int count;
    private final int delay;
    private int cooldown;
    private int used;
    private int pending = -1;
    private long pendingBoost;

    private FireworkSchedule(long boost, int[] flights, int count, int cooldown, int delay) {
        this.boost = boost;
        this.flights = flights.clone();
        this.count = count;
        this.cooldown = Math.max(0, cooldown);
        this.delay = delay;
    }

    public static long minimumBoostTicks(int flight) {
        // Boost precedes ++life > lifeTime, including the expiry tick; random minima.
        return 10L * (1 + Math.max(0L, flight)) + 1;
    }

    static FireworkSchedule consecutive(long boostTicks) {
        return new FireworkSchedule(boostTicks, new int[0], 0, 0, 0);
    }

    public static FireworkSchedule confirmed(int[] flights, int count, int cooldown,
            long confirmedBoost, int confirmationDelay) {
        if (count < 0 || count > flights.length || confirmedBoost < 0 || confirmationDelay < 0)
            throw new IllegalArgumentException("Invalid firework schedule");
        return new FireworkSchedule(confirmedBoost, flights, count, cooldown, confirmationDelay);
    }

    boolean nextPowered() {
        return nextTick().powered();
    }

    public Tick nextTick() {
        return nextTick(true);
    }

    /** Suppressing a use still ages existing attachment, pending acknowledgement and cooldown. */
    public Tick nextTick(boolean allowUse) {
        boolean requested = false;
        if (allowUse && boost == 0 && cooldown == 0 && pending < 0 && used < count) {
            requested = true;
            // A spawned rocket ages while its acknowledgement is in flight.
            pendingBoost = Math.max(0, minimumBoostTicks(flights[used++]) - delay);
            pending = delay;
            cooldown = 40;
        }
        if (pending == 0) {
            boost = pendingBoost;
            pendingBoost = 0;
            pending = -1;
        }
        boolean powered = boost > 0;
        boost = Math.max(0, boost - 1);
        cooldown = Math.max(0, cooldown - 1);
        if (pending > 0) pending--;
        return new Tick(requested, powered, checkpoint());
    }

    /** Conservative reserve work budget, not a touchdown witness. Only acknowledged
     * powered ticks earn distance; coasting earns no distance and each unpowered tick
     * adds a small climb/recovery allowance. Missing inventory is extrapolated with the
     * ordinary flight=1 component so fuel shortage remains distinct from estimate failure.
     * Existing inventory is always consumed in its actual order.
     */
    public static int reserveRockets(int[] inventory, int cooldown, long confirmedBoost,
            int confirmationDelay, double distance, double climb, double reversalSpeed) {
        if (!Double.isFinite(distance + climb + reversalSpeed) || distance < 0 || climb < 0
                || cooldown > 600 || confirmationDelay >= minimumBoostTicks(1)) return -1;
        int[] forecast = java.util.Arrays.copyOf(inventory, Math.max(128, inventory.length));
        java.util.Arrays.fill(forecast, inventory.length, forecast.length, 1);
        var schedule = confirmed(forecast, forecast.length, cooldown, confirmedBoost, confirmationDelay);
        // Deliberately discount powered cruise (~1.7 blocks/tick), charge reversal,
        // and retain a separate approach allowance even for a nearby anchor.
        double work = distance / 1.25 + climb * 3 + Math.max(0, reversalSpeed) * 12 + 12;
        for (int tick = 0; tick < 12000; tick++) {
            if (Thread.currentThread().isInterrupted()) return -1;
            var next = schedule.nextTick();
            work += next.powered() ? -1 : .05;
            if (work <= 0) return next.checkpoint().consumed();
            if (schedule.exhausted()) return -1;
        }
        return -1;
    }

    boolean exhausted() { return boost == 0 && pending < 0 && used >= count; }
}
