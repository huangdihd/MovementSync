package xin.bbtt.physics;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.joml.Vector3d;
import xin.bbtt.world.World;

/** Worker-owned geometric guidance, not a flight-control or recovery certificate.
 * The route snapshot never follows the movement-thread target's mutable cursor.
 * All exposed vectors are copies. Calls and the arc-length cursor are single-threaded.
 */
public final class LocalAirRoute {
    private static final double MAX_LOOKAHEAD = 48;
    private static final double SAMPLE_SPACING = 2;
    private static final double EPSILON = 1.0e-8;
    private final List<Vector3d> points;
    private final double[] arcs;
    private double cursor;

    public LocalAirRoute(WaypointFlightTarget target) {
        points = Objects.requireNonNull(target).waypoints();
        arcs = new double[points.size()];
        for (int i = 1; i < arcs.length; i++) {
            arcs[i] = arcs[i - 1] + points.get(i - 1).distance(points.get(i));
            if (!Double.isFinite(arcs[i])) throw new IllegalArgumentException("Route length overflow");
        }
        // The target index names the active segment's endpoint. Start at that segment,
        // never reintroduce waypoints already consumed by the movement-thread cursor.
        cursor = arcs[Math.max(0, target.index() - 1)];
    }

    public double cursor() { return cursor; }

    /** Equal-geometry renewal may advance the movement-thread segment, but never rewind
     * the worker's finer arc-length progress within or beyond that segment. */
    public void advanceToLiveSegment(WaypointFlightTarget target) {
        if (!points.equals(Objects.requireNonNull(target).waypoints()))
            throw new IllegalArgumentException("Different route geometry");
        cursor = Math.max(cursor, arcs[Math.max(0, target.index() - 1)]);
    }

    public double length() { return arcs[arcs.length - 1]; }
    public List<Vector3d> waypoints() { return points.stream().map(Vector3d::new).toList(); }

    /** Farthest-first visible steering alternatives ahead of the forward projection.
     * Empty means no loaded body-visible alternative, never permission to cross a wall.
     * Distance, speed and a nominal 10-degree/tick turn radius bound the local search;
     * this radius is guidance only, not a claim of dynamically feasible turning.
     * The actual state's pose supplies the clearance, including on-ground handoff.
     */
    public List<Vector3d> lookahead(World world, PhysicsState state) {
        Objects.requireNonNull(world);
        Objects.requireNonNull(state);
        Vector3d position = state.position();
        double speed = state.velocity().length();
        if (!position.isFinite() || !Double.isFinite(speed)) return List.of();
        double radius = speed / Math.toRadians(10);
        double reach = Math.min(MAX_LOOKAHEAD, Math.max(4, 6 * speed + radius));
        project(world, position, state.pose(), reach);
        double end = Math.min(length(), cursor + reach);
        List<Double> candidates = new ArrayList<>();
        candidates.add(end);
        for (double arc = end - SAMPLE_SPACING; arc > cursor + EPSILON; arc -= SAMPLE_SPACING)
            candidates.add(arc);
        // Preserve sharp corners even when the regular sampling misses their vertex.
        for (double arc : arcs) if (arc > cursor + EPSILON && arc < end) candidates.add(arc);
        candidates.sort(java.util.Comparator.reverseOrder());
        List<Vector3d> result = new ArrayList<>();
        double previous = Double.NaN;
        for (double arc : candidates) {
            if (arc == previous) continue;
            previous = arc;
            Vector3d candidate = at(arc);
            if (position.distance(candidate) <= MAX_LOOKAHEAD
                    && visible(world, position, candidate, state.pose())) result.add(candidate);
        }
        return List.copyOf(result);
    }

    private void project(World world, Vector3d position, PlayerPose pose, double reach) {
        double bestArc = cursor;
        double bestDistance = position.distanceSquared(at(cursor));
        double limit = Math.min(length(), cursor + Math.max(reach, position.distance(at(cursor))));
        for (int i = 1; i < points.size(); i++) {
            double segment = arcs[i] - arcs[i - 1];
            if (segment == 0 || arcs[i] < cursor || arcs[i - 1] > limit) continue;
            Vector3d direction = new Vector3d(points.get(i)).sub(points.get(i - 1));
            double t = new Vector3d(position).sub(points.get(i - 1)).dot(direction) / (segment * segment);
            double arc = Math.max(cursor, Math.min(limit, arcs[i - 1] + Math.max(0, Math.min(1, t)) * segment));
            Vector3d projected = at(arc);
            double distance = position.distanceSquared(projected);
            // Ties keep the earlier arc, so crossing/duplicate vertices cannot teleport progress.
            if (distance < bestDistance - EPSILON && visible(world, position, projected, pose)) {
                bestDistance = distance;
                bestArc = arc;
            }
        }
        cursor = bestArc;
    }

    private Vector3d at(double arc) {
        for (int i = 1; i < points.size(); i++) {
            if (arcs[i] < arc || arcs[i] == arcs[i - 1]) continue;
            return new Vector3d(points.get(i - 1)).lerp(points.get(i),
                    (arc - arcs[i - 1]) / (arcs[i] - arcs[i - 1]));
        }
        return new Vector3d(points.get(points.size() - 1));
    }

    /** Conservative continuous body sweep against exact World collision shapes.
     * No point-ray/solid-block approximation and no tunnelling between samples.
     * Recursive enclosing AABBs cover the entire sweep. Ambiguous sub-millimetre
     * grazing and exhausted query budgets fail closed. Unknown chunks are solid.
     * Zero-length rays still validate the complete body box.
     */
    public static boolean visible(World world, Vector3d from, Vector3d to, PlayerPose pose) {
        Objects.requireNonNull(world);
        Objects.requireNonNull(pose);
        if (!from.isFinite() || !to.isFinite()) return false;
        // Keep chunk scans and collision queries bounded even for external callers.
        if (from.distance(to) > MAX_LOOKAHEAD) return false;
        return sweep(world, from, to, pose.height(), 0, new int[] {8192});
    }

    private static boolean sweep(World world, Vector3d a, Vector3d b, double height,
            int depth, int[] budget) {
        if (--budget[0] < 0) return false;
        double r = PlayerPose.HALF_WIDTH;
        double x = Math.min(a.x, b.x) - r, X = Math.max(a.x, b.x) + r;
        double y = Math.min(a.y, b.y), Y = Math.max(a.y, b.y) + height;
        double z = Math.min(a.z, b.z) - r, Z = Math.max(a.z, b.z) + r;
        for (int cx = (int)Math.floor(x / 16); cx <= (int)Math.floor(Math.nextDown(X) / 16); cx++)
            for (int cz = (int)Math.floor(z / 16); cz <= (int)Math.floor(Math.nextDown(Z) / 16); cz++)
                if (!world.chunkLoaded(cx, cz)) return false;
        if (!world.isBoxColliding(x, y, z, X, Y, Z)) return true;
        if (depth == 20 || a.distanceSquared(b) <= 1.0e-8) return false;
        Vector3d middle = new Vector3d(a).lerp(b, .5);
        return sweep(world, a, middle, height, depth + 1, budget)
                && sweep(world, middle, b, height, depth + 1, budget);
    }
}
