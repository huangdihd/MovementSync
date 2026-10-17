package xin.bbtt.physics;

import org.joml.Vector3d;
import xin.bbtt.world.World;
import static xin.bbtt.physics.VanillaGlideMode.*;

/** Internal legacy cruise/approach costs. Arithmetic and accumulation order are intentional. */
final class BuiltinFlightCost {
    static double controls(float candidateYaw, float candidatePitch, float yaw, float pitch, boolean firework) {
        return Math.abs(normalize(candidateYaw - yaw)) * 0.015
                + Math.abs(candidatePitch - pitch) * 0.01 + (firework ? 6 : 0);
    }

    static double emergencyFallback(float candidateYaw, float candidatePitch,
            float yaw, float heading, boolean rememberedTarget) {
        return Math.abs(normalize(candidateYaw - (rememberedTarget ? heading : yaw))) * .015
                + Math.abs(candidatePitch + 10) * .01;
    }

    static double emergencyControls(float candidateYaw, float candidatePitch, float yaw, float pitch) {
        return Math.abs(normalize(candidateYaw - yaw)) * .015
                + Math.abs(candidatePitch - pitch) * .01;
    }

    static double rememberedRange(GlideRange glide, double height, float yaw, float pitch, float heading) {
        return -glide.projectedRange(height, yaw, pitch)
                * Math.cos(Math.toRadians(normalize(yaw - heading)));
    }

    static double evaluate(World world, FlightTrajectory trajectory, Vector3d target,
            double radius, double cruiseAltitude, float yaw, float pitch,
            boolean boosted, boolean landing, boolean forcedLanding, int horizon, boolean emergencyApproach) {
        PhysicsState state = trajectory.initial();
        double score = 0;
        for (int tick = 0; tick < horizon; tick++) {
            PhysicsResult result = trajectory.result(tick);
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
                if (forcedLanding) {
                    return Math.abs(state.velocity().y) <= 0.5
                            && Math.hypot(state.velocity().x, state.velocity().z) <= 0.8
                            ? impact * 1000 + distance : Double.POSITIVE_INFINITY;
                }
                return landing && distance <= radius && Math.abs(position.y - target.y) <= 0.5
                        && Math.abs(state.velocity().y) <= 0.5
                        && Math.hypot(state.velocity().x, state.velocity().z) <= 0.8
                        ? score + impact * 30
                        : Double.POSITIVE_INFINITY;
            }
            // A short horizon must not approve sinking below an island before reaching it.
            // Keep the emergency glide envelope viable until there is solid ground below.
            if (emergencyApproach && !boosted && position.y < target.y + distance / 5
                    && !supportedLandingArea(world, position.x, target.y, position.z)) {
                return Double.POSITIVE_INFINITY;
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
            state = new PhysicsState(position, result.velocity(), false, result.fallDistance());
        }
        if (forcedLanding && horizon == HORIZON && !emergencyApproach) return Double.POSITIVE_INFINITY;
        Vector3d position = state.position();
        if (!landing) return score + horizontalDistance(position, target) * 3;
        return score + horizontalDistance(position, target) * 4
                + Math.abs(position.y - target.y) * 3 + state.velocity().length() * 12;
    }

}
