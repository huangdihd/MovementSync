package xin.bbtt.flight;

import org.joml.Vector3d;
import xin.bbtt.physics.FlightHazards;
import xin.bbtt.physics.HazardRecovery;
import xin.bbtt.physics.KnownLandingReserve;
import xin.bbtt.physics.ElytraFlightController;
import xin.bbtt.physics.MovementParameters;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerPose;
import xin.bbtt.world.World;

/** Policy-specific gate for a selected prefix entering dangerous terrain.
 * Invoke synchronously on the controller worker, outside the physics lock, once
 * per chosen prefix, never while ranking candidates. Both policies use real remaining
 * inventory; legacy entry requires recovery proof, known-return entry a resource estimate.
 */
public final class DangerousEntryGate {
    private DangerousEntryGate() {}

    /** Every footprint column must have known solid, nonhot support below it.
     * Unknown terrain, hot cells before support, and the unsupported void are exposed.
     */
    public static boolean exposed(World world, PhysicsState state) {
        Vector3d feet = state.position();
        if (!feet.isFinite() || feet.y < World.getMinWorldY()
                || feet.y > World.getMaxWorldY()) return true;
        boolean[] exposed = {false};
        world.readTerrain(() -> {
            double radius = PlayerPose.HALF_WIDTH;
            for (int x = (int) Math.floor(feet.x - radius); x <= Math.floor(feet.x + radius); x++) {
                for (int z = (int) Math.floor(feet.z - radius); z <= Math.floor(feet.z + radius); z++) {
                    if (!world.chunkLoaded(x >> 4, z >> 4)) {
                        exposed[0] = true;
                        return false;
                    }
                    boolean supported = false;
                    for (int y = (int) Math.floor(feet.y); y >= World.getMinWorldY(); y--) {
                        Vector3d cell = new Vector3d(x + .5, y, z + .5);
                        if (!FlightHazards.safeLanding(world, new Vector3d(cell).add(0, 1, 0))) {
                            exposed[0] = true;
                            return false;
                        }
                        if (world.isSolid(cell)) {
                            supported = true;
                            break;
                        }
                    }
                    if (!supported) {
                        exposed[0] = true;
                        return false;
                    }
                }
            }
            return true;
        });
        return exposed[0];
    }

    /** Context must describe resources remaining at the chosen prefix's end.
     * Already exposed states are managed by the recovery controller, not repeatedly
     * gated here. New entry requires a finite proof; exhaustion fails closed.
     */
    public static boolean permit(World world, PhysicsState before, PhysicsState end,
            MovementParameters movement, HazardRecovery.Context context) {
        return permit(world, before, end, movement, context, false);
    }

    /** New entry requires a valid known-supported-site estimate with positive surplus. */
    public static boolean permitKnownReturn(World world, PhysicsState before, PhysicsState end,
            MovementParameters movement, HazardRecovery.Context context) {
        return permit(world, before, end, movement, context, true);
    }

    private static boolean permit(World world, PhysicsState before, PhysicsState end,
            MovementParameters movement, HazardRecovery.Context context, boolean knownReturn) {
        if (exposed(world, before) || !exposed(world, end)) return true;
        if (!knownReturn) {
            HazardRecovery.Budget budget = new HazardRecovery.Budget(HazardRecovery.DECISION_STEP_BUDGET);
            HazardRecovery.Report report = HazardRecovery.assess(world, end, movement, context, budget);
            return !budget.exhausted() && report.status() == HazardRecovery.Status.RECOVERABLE;
        }
        return KnownLandingReserve.assess(world, end, movement, context,
                knownSites(world, end, context), 10, 1).canContinue();
    }

    /** At most 148 unique observations: 66 memory, underfoot, and all 81 loaded probes.
     * Fresh observations have an independent allowance, even with full memory. */
    public static java.util.List<Vector3d> knownSites(World world, PhysicsState state,
            HazardRecovery.Context context) {
        var sites = new java.util.LinkedHashSet<Vector3d>();
        if (context.memory() != null) {
            sites.addAll(context.memory().snapshot());
        }
        var p = state.position();
        var below = ElytraFlightController.landingSite(world, p.x, p.z, p.y);
        if (below != null) sites.add(below);
        for (int dx = -32; dx <= 32; dx += 8) for (int dz = -32; dz <= 32; dz += 8) {
            var site = ElytraFlightController.landingSite(world, p.x + dx, p.z + dz, p.y);
            if (site != null) sites.add(site);
        }
        return java.util.List.copyOf(sites);
    }
}
