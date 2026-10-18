package xin.bbtt.physics;

import java.util.Comparator;
import java.util.HashMap;
import java.util.PriorityQueue;
import org.joml.Vector3d;
import xin.bbtt.world.World;

/** Movement-owned incremental nearest-first loaded-chunk search. */
public final class LandingSearch {
    // Engineering budget: 512 queue visits/tick, at most 512 terrain columns
    // plus direct-below and previous-target checks. Sixteen samples/chunk
    // covers nearby (~48 block) islands in one decision, while distant void
    // scans are spread over ticks instead of querying all 441 view chunks.
    static final int BUDGET = 512;
    private record Entry(double distance, int x, int z, boolean chunk) {}
    private final PriorityQueue<Entry> pending = new PriorityQueue<>(Comparator.comparingDouble(Entry::distance));
    private Vector3d origin;
    private World owner;
    private Vector3d nearest;
    private Vector3d previous;
    private Vector3d remembered;
    private boolean complete = true;

    /**
     * Selects a supported, hazard-free landing site at or below the current altitude
     * using the ordinary nearest-first loaded-chunk samples and glide-range checks.
     * The entire pending sweep is available in this call; every returned site must
     * also satisfy {@code reachable}. Corridor callers should accept only complete
     * (non-partial) geometric routes that reach the candidate site.
     *
     * @param reachable additional reachability check, required for every candidate
     * @return an accepted landing site, or {@code null} when none is available
     */
    public Vector3d selectReachable(World world, PhysicsState state, int[] flights,
            boolean attached, MovementParameters parameters,
            java.util.function.Predicate<Vector3d> reachable) {
        java.util.Objects.requireNonNull(reachable, "reachable");
        return select(world, state, flights, attached, parameters, Integer.MAX_VALUE, reachable, true);
    }

    /** Work-bounded terminal-proof candidate search; rejection never certifies safety. */
    Vector3d selectReachable(World world, PhysicsState state, int[] flights,
            boolean attached, MovementParameters parameters, int budget,
            java.util.function.Predicate<Vector3d> reachable) {
        return select(world, state, flights, attached, parameters, budget, reachable, true);
    }

    Vector3d select(World world, PhysicsState state, int[] flights, boolean attached,
            MovementParameters parameters, int budget) {
        return select(world, state, flights, attached, parameters, budget, null, false);
    }

    private Vector3d select(World world, PhysicsState state, int[] flights, boolean attached,
            MovementParameters parameters, int budget,
            java.util.function.Predicate<Vector3d> filter, boolean low) {
        if (!state.position().isFinite() || !state.velocity().isFinite())
            throw new IllegalArgumentException("Flight state must be finite");
        Vector3d position = state.position();
        // Do not choose the far side of a solid roof from inside a ceiling dimension.
        double searchCeiling = low || xin.bbtt.listeners.RegistryDataListener.hasCeiling()
                && !CeilingCollision.isAboveRoof(world, position)
                ? position.y : World.getMaxWorldY();
        Vector3d below = VanillaGlideMode.landingSite(world, position.x, position.z, position.y);
        if (below != null && (filter == null || filter.test(below))) {
            complete = true; previous = below; return below;
        }
        // Finish a sweep while moving to avoid starving distant islands at
        // chunk boundaries. Ordering is relative to the fixed sweep origin;
        // reachability always uses the current state. Refresh after completion.
        if (complete || owner != world) {
            pending.clear();
            origin = new Vector3d(position);
            owner = world;
            nearest = null;
            complete = false;
            for (var c : world.loadedChunks()) {
                double x = Math.max(c.x() * 16, Math.min(position.x, c.x() * 16 + 16));
                double z = Math.max(c.z() * 16, Math.min(position.z, c.z() * 16 + 16));
                pending.add(new Entry(Math.hypot(x - position.x, z - position.z), c.x(), c.z(), true));
            }
        }
        GlideRange range = new GlideRange(state, parameters, VanillaGlideMode.landingPitches());
        var ranges = new HashMap<Double, Double>();
        for (int visits = 0; visits < budget && !pending.isEmpty(); visits++) {
            if (filter != null && Thread.currentThread().isInterrupted()) return null;
            Entry e = pending.remove();
            if (e.chunk) {
                if (!world.chunkLoaded(e.x, e.z)) continue;
                // Expanding chunks by their distance lower bound is a circular
                // chunk-ring traversal. Samples compete with unopened rings, so
                // the first reachable sample is nearest, including chunk edges.
                int ox = Math.floorMod((int) Math.floor(origin.x), 4);
                int oz = Math.floorMod((int) Math.floor(origin.z), 4);
                for (int x = e.x * 16 + ox; x < e.x * 16 + 16; x += 4)
                    for (int z = e.z * 16 + oz; z < e.z * 16 + 16; z += 4)
                        pending.add(new Entry(Math.hypot(x + .5 - origin.x, z + .5 - origin.z), x, z, false));
                continue;
            }
            Vector3d site = VanillaGlideMode.landingSite(world, e.x + .5, e.z + .5, searchCeiling);
            if (site == null || filter != null && !filter.test(site)) continue;
            if (nearest == null) nearest = site;
            if (Math.hypot(position.x - site.x, position.z - site.z)
                    <= ranges.computeIfAbsent(site.y, y -> range.atHeight(y, flights, attached))) {
                complete = true;
                previous = site;
                return new Vector3d(site);
            }
        }
        complete = pending.isEmpty();
        if (complete && nearest != null && (flights.length > 0 || attached)) {
            Vector3d valid = VanillaGlideMode.landingSite(world, nearest.x, nearest.z, searchCeiling);
            if (valid != null && (filter == null || filter.test(valid))) {
                previous = valid;
                return valid;
            }
        }
        if (!complete && previous != null) {
            Vector3d valid = VanillaGlideMode.landingSite(world, previous.x, previous.z, searchCeiling);
            if (valid != null && valid.y == previous.y
                    && (filter == null || filter.test(valid))) return valid;
        }
        return null;
    }

    void selectedMemory(Vector3d site) {
        if (site != null && !site.equals(remembered))
            xin.bbtt.MovementSync.getLogger().info("Emergency landing using remembered ground {}", site);
        remembered = site == null ? null : new Vector3d(site);
    }

}
