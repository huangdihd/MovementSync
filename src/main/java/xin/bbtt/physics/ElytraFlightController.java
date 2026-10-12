package xin.bbtt.physics;

import java.util.Objects;
import org.joml.Vector3d;
import xin.bbtt.world.World;

/** Stateless, bounded trajectory steering. The caller owns rocket lifetime and flight lifecycle. */
public final class ElytraFlightController {
    private static final int HORIZON = 24;
    private static final double HALF_WIDTH = 0.299;
    private static final double HEIGHT = 0.6;
    private static final float[] YAW_OFFSETS = {0, -30, 30, -60, 60, -100, 100, 180};
    // Cancellation freezes steering. Avoid dives whose unpowered terminal sink
    // exceeds limitFallDistance's -0.5 threshold; descend by a shallow glide instead.
    private static final float[] PITCHES = {-55, -30, -10, 0, 10};

    public enum Phase { CLIMB, CRUISE, DESCENT, AVOIDANCE, LANDING, LANDED }

    public record Decision(float yaw, float pitch, boolean firework, Phase phase) {}

    public Decision decide(World world, PhysicsState state, float yaw, float pitch,
            boolean attached, Vector3d target, int fireworks, double radius) {
        Objects.requireNonNull(world, "world");
        Objects.requireNonNull(state, "state");
        Objects.requireNonNull(target, "target");
        if (!state.position().isFinite() || !state.velocity().isFinite() || !target.isFinite()
                || !Float.isFinite(yaw) || !Float.isFinite(pitch)
                || !Double.isFinite(radius) || radius <= 0 || fireworks < 0) {
            throw new IllegalArgumentException("Flight inputs must be finite, radius positive and fireworks nonnegative");
        }
        if (state.onGround()) return new Decision(normalize(yaw), 0, false, Phase.LANDED);

        Vector3d position = state.position();
        double distance = horizontalDistance(position, target);
        double speed = Math.hypot(state.velocity().x, state.velocity().z);
        double cruiseAltitude = routeAltitude(world, position, target);
        boolean descending = distance <= Math.max(48, (position.y - target.y) * 5 + speed * HORIZON);
        boolean landing = descending && distance <= Math.max(radius + 16, speed * HORIZON)
                && position.y <= target.y + 12;
        Phase phase = Phase.CRUISE;
        if (landing) phase = Phase.LANDING;
        else if (descending) phase = Phase.DESCENT;
        else if (position.y < cruiseAltitude - 3) phase = Phase.CLIMB;
        double desiredAltitude = descending
                ? Math.min(cruiseAltitude, target.y + Math.max(0, distance - radius) / 5)
                : cruiseAltitude;
        boolean needsBoost = !descending && (position.y < cruiseAltitude - 3 || speed < 0.65);
        float heading = distance < 0.01 ? normalize(yaw)
                : (float) Math.toDegrees(Math.atan2(position.x - target.x, target.z - position.z));
        Decision best = null;
        double bestScore = Double.POSITIVE_INFINITY;
        for (float offset : YAW_OFFSETS) {
            // Entity.changeLookDirection accumulates yaw; wrapping the wire angle
            // at +/-180 creates an artificial full-turn discontinuity.
            float candidateYaw = yaw + normalize(heading + offset - yaw);
            for (float candidatePitch : PITCHES) {
                // Attached rockets are predicted, never requested a second time. New rockets
                // are compared against coasting, not automatically spent on every decision.
                int boostOptions = !attached && fireworks > 0 && needsBoost ? 2 : 1;
                for (int boost = 0; boost < boostOptions; boost++) {
                    boolean firework = boost == 1;
                    double score = evaluate(world, state, target, radius, desiredAltitude,
                            candidateYaw, candidatePitch, attached || firework, landing);
                    if (!Double.isFinite(score)) continue;
                    score += Math.abs(normalize(candidateYaw - yaw)) * 0.015
                            + Math.abs(candidatePitch - pitch) * 0.01 + (firework ? 6 : 0);
                    if (best != null && score >= bestScore) continue;
                    bestScore = score;
                    best = new Decision(candidateYaw, candidatePitch, firework, phase);
                }
            }
        }
        if (best == null) throw new IllegalStateException("No collision-free flight trajectory");
        return best;
    }

    private static double evaluate(World world, PhysicsState initial, Vector3d target,
            double radius, double cruiseAltitude, float yaw, float pitch,
            boolean boosted, boolean landing) {
        PhysicsState state = initial;
        GlidingPhysics physics = new GlidingPhysics(yaw, pitch, boosted);
        double score = 0;
        for (int tick = 0; tick < HORIZON; tick++) {
            PhysicsResult result = physics.step(world, state, HALF_WIDTH, HEIGHT);
            Vector3d previous = state.position();
            Vector3d position = result.position();
            double distance = horizontalDistance(position, target);
            // Unknown space is not air. Turn/climb inside loaded chunks until
            // streaming reveals the next section; never approve blind low flight.
            for (int chunkX = (int) Math.floor(Math.min(previous.x, position.x) - HALF_WIDTH) >> 4;
                    chunkX <= (int) Math.floor(Math.max(previous.x, position.x) + HALF_WIDTH) >> 4; chunkX++) {
                for (int chunkZ = (int) Math.floor(Math.min(previous.z, position.z) - HALF_WIDTH) >> 4;
                        chunkZ <= (int) Math.floor(Math.max(previous.z, position.z) + HALF_WIDTH) >> 4; chunkZ++) {
                    if (!world.chunkLoaded(chunkX, chunkZ)) return Double.POSITIVE_INFINITY;
                }
            }
            if (result.collidedX() || result.collidedZ()) return Double.POSITIVE_INFINITY;
            // Physics clips axes independently. Check the swept envelope too, so a fast
            // candidate cannot tunnel through thin obstacles between tick endpoints.
            if (world.isBoxColliding(Math.min(previous.x, position.x) - HALF_WIDTH,
                    Math.min(previous.y, position.y), Math.min(previous.z, position.z) - HALF_WIDTH,
                    Math.max(previous.x, position.x) + HALF_WIDTH,
                    Math.max(previous.y, position.y) + HEIGHT,
                    Math.max(previous.z, position.z) + HALF_WIDTH)) {
                return Double.POSITIVE_INFINITY;
            }
            if (result.onGround()) {
                double impact = Math.abs(state.velocity().y) + Math.hypot(state.velocity().x, state.velocity().z);
                return landing && distance <= radius && Math.abs(position.y - target.y) <= 0.5
                        && Math.abs(state.velocity().y) <= 0.5
                        && Math.hypot(state.velocity().x, state.velocity().z) <= 0.8
                        ? score + impact * 30
                        : Double.POSITIVE_INFINITY;
            }
            if (position.y < world.getMinWorldY() || position.y + HEIGHT > world.getMaxWorldY()) {
                return Double.POSITIVE_INFINITY;
            }
            if (!landing && world.isBoxColliding(position.x - HALF_WIDTH - 1,
                    position.y - 3, position.z - HALF_WIDTH - 1,
                    position.x + HALF_WIDTH + 1, position.y + HEIGHT + 1,
                    position.z + HALF_WIDTH + 1)) score += 100;
            score += landing ? (distance + Math.abs(position.y - target.y) * 1.5) / HORIZON
                    : Math.abs(position.y - cruiseAltitude) * 0.12;
            state = new PhysicsState(position, result.velocity(), false);
        }
        Vector3d position = state.position();
        if (!landing) return score + horizontalDistance(position, target) * 3;
        return score + horizontalDistance(position, target) * 4
                + Math.abs(position.y - target.y) * 3 + state.velocity().length() * 12;
    }

    /** Scan bounded route tiles, never treating unloaded terrain as known air. */
    private static double routeAltitude(World world, Vector3d position, Vector3d target) {
        double distance = horizontalDistance(position, target);
        int tiles = Math.min(64, Math.max(1, (int) Math.ceil(distance / 16)));
        double scannedDistance = Math.min(distance, 1024);
        double highest = target.y;
        boolean unknown = distance > scannedDistance;
        for (int tile = 0; tile < tiles; tile++) {
            double start = distance == 0 ? 0 : scannedDistance * tile / tiles / distance;
            double end = distance == 0 ? 0 : scannedDistance * (tile + 1) / tiles / distance;
            double firstX = position.x + (target.x - position.x) * start;
            double firstZ = position.z + (target.z - position.z) * start;
            double lastX = position.x + (target.x - position.x) * end;
            double lastZ = position.z + (target.z - position.z) * end;
            double minX = Math.min(firstX, lastX) - HALF_WIDTH;
            double maxX = Math.max(firstX, lastX) + HALF_WIDTH;
            double minZ = Math.min(firstZ, lastZ) - HALF_WIDTH;
            double maxZ = Math.max(firstZ, lastZ) + HALF_WIDTH;
            for (int chunkX = (int) Math.floor(minX) >> 4; chunkX <= (int) Math.floor(maxX) >> 4; chunkX++) {
                for (int chunkZ = (int) Math.floor(minZ) >> 4; chunkZ <= (int) Math.floor(maxZ) >> 4; chunkZ++) {
                    if (!world.chunkLoaded(chunkX, chunkZ)) {
                        unknown = true;
                        continue;
                    }
                    highest = Math.max(highest, world.findHighestCollisionTop(
                            Math.max(minX, chunkX * 16.0), World.getMinWorldY(),
                            Math.max(minZ, chunkZ * 16.0), Math.min(maxX, chunkX * 16.0 + 16),
                            World.getMaxWorldY(), Math.min(maxZ, chunkZ * 16.0 + 16)).orElse(highest));
                }
            }
        }
        // Distant unknown terrain raises the overworld reserve, not a global stop.
        // evaluate still rejects any trajectory entering an unloaded chunk at any altitude.
        double altitude = highest + 16;
        if (unknown) altitude = Math.max(altitude, Math.max(128, target.y + 32));
        return Math.min(World.getMaxWorldY() - HEIGHT - 2, altitude);
    }

    private static double horizontalDistance(Vector3d first, Vector3d second) {
        return Math.hypot(first.x - second.x, first.z - second.z);
    }

    private static float normalize(float angle) {
        float wrapped = angle % 360;
        if (wrapped >= 180) return wrapped - 360;
        if (wrapped < -180) return wrapped + 360;
        return wrapped;
    }
}
