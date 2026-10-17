package xin.bbtt.movements;

import java.util.LinkedHashMap;
import org.joml.Vector3d;
import xin.bbtt.physics.ElytraFlightController;
import xin.bbtt.world.World;

/** Session-owned flight observations, retained across navigation cancellation. */
public final class LandingMemory {
    // Engineering bounds: one representative per 8x8 chunks (128 blocks), 64 cells.
    // This avoids spending the budget on adjacent columns; insertion age eviction
    // retains about 8 km of a straight route, independently of observation frequency.
    private final LinkedHashMap<Long, Vector3d> sites = new LinkedHashMap<>();
    private Vector3d takeoff;
    private Vector3d lastLanding;
    private String dimension;
    private int sampleOffset;

    public synchronized void clear() {
        sites.clear();
        takeoff = null;
        lastLanding = null;
        sampleOffset = 0;
    }

    public synchronized void dimension(String value) {
        if (!java.util.Objects.equals(dimension, value)) clear();
        dimension = value;
    }

    /** Pin the first valid departure ground until the session/dimension is cleared. */
    public synchronized void takeoff(World world, Vector3d position) {
        if (takeoff == null)
            takeoff = ElytraFlightController.landingSite(world, position.x, position.z, position.y);
    }

    /** Called only on actual solid-ground touchdown, not on nearby ground observations. */
    public synchronized void landed(World world, Vector3d position) {
        Vector3d site = ElytraFlightController.landingSite(world, position.x, position.z, position.y);
        if (site != null) lastLanding = site;
    }

    public synchronized void observe(World world, Vector3d position) {
        remember(ElytraFlightController.landingSite(world, position.x, position.z, position.y));
        // Engineering budget: one underfoot + at most nine nearby probes per observation.
        // Start at each chunk's centre; rotate through all 16x16 column offsets in
        // 256 observations so a stationary observer also discovers off-centre ground.
        int chunkX = (int) Math.floor(position.x) >> 4;
        int chunkZ = (int) Math.floor(position.z) >> 4;
        double offsetX = ((8 + sampleOffset % 16) % 16) + .5;
        double offsetZ = ((8 + sampleOffset / 16) % 16) + .5;
        sampleOffset = (sampleOffset + 1) % 256;
        for (int x = chunkX - 1; x <= chunkX + 1; x++) {
            for (int z = chunkZ - 1; z <= chunkZ + 1; z++) {
                if (!world.chunkLoaded(x, z)) continue;
                remember(ElytraFlightController.landingSite(world,
                        x * 16.0 + offsetX, z * 16.0 + offsetZ, position.y));
            }
        }
    }

    private void remember(Vector3d site) {
        // Unknown/unloaded terrain must not invalidate an earlier observation.
        if (site == null) return;
        int x = (int) Math.floor(site.x / 128);
        int z = (int) Math.floor(site.z / 128);
        long key = ((long) x << 32) | (z & 0xffffffffL);
        sites.putIfAbsent(key, new Vector3d(site));
        while (sites.size() > 64) sites.remove(sites.keySet().iterator().next());
    }

    public synchronized Vector3d nearest(Vector3d position) {
        return java.util.stream.Stream.concat(sites.values().stream(),
                java.util.stream.Stream.of(takeoff, lastLanding).filter(java.util.Objects::nonNull))
                .min(java.util.Comparator.comparingDouble(site ->
                Math.hypot(site.x - position.x, site.z - position.z)))
                .map(Vector3d::new).orElse(null);
    }
}
