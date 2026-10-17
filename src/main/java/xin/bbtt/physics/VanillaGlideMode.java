package xin.bbtt.physics;

import java.util.OptionalDouble;
import java.util.Objects;
import org.joml.Vector3d;
import xin.bbtt.world.World;

/** Internal original glide mode: ordered controls, physics and verified landing geometry. */
final class VanillaGlideMode implements FlightMode {
    static final int HORIZON = 24;
    private final MovementParameters movement;
    final LandingSearch landingSearch = new LandingSearch();
    GlideRange range(PhysicsState state) { return new GlideRange(state, movement, PITCHES); }
    VanillaGlideMode(MovementParameters movement) { this.movement = movement; }
    static final double HALF_WIDTH = 0.299;
    static final double HEIGHT = 0.6;
    static final float[] YAW_OFFSETS = {0, -30, 30, -60, 60, -100, 100, 180};
    // Cancellation freezes steering. Avoid dives whose unpowered terminal sink
    // exceeds limitFallDistance's -0.5 threshold; descend by a shallow glide instead.
    static final float[] PITCHES = {-55, -30, -10, 0, 10};

    @Override public PhysicsStepper stepper(float yaw, float pitch, boolean boosted) {
        return new GlidingPhysics(yaw, pitch, boosted, movement.gravity(), movement.slowFalling());
    }

    @Override public FlightSafety.RecoveryStrategy recovery(FlightSafety safety) {
        return new VanillaGlideRecovery(safety, PITCHES);
    }

    @Override public java.util.List<Candidate> candidates(float yaw, float heading,
            boolean compareBoost, boolean firework) {
        var candidates = new java.util.ArrayList<Candidate>();
        for (float offset : YAW_OFFSETS) {
            // Preserve accumulated wire yaw rather than introducing a wrapped full turn.
            float candidateYaw = yaw + normalize(heading + offset - yaw);
            for (float pitch : PITCHES) {
                for (int boost = 0; boost < (compareBoost ? 2 : 1); boost++) {
                    candidates.add(new Candidate(this, candidateYaw, pitch, firework || boost == 1));
                }
            }
        }
        return candidates;
    }

    /** Selects the nearest physics-reachable sampled site in loaded terrain. */
    public static Vector3d selectForcedLandingTarget(World world, PhysicsState state,
            int fireworks, boolean attached) {
        return selectForcedLandingTarget(world, state, fireworks, attached, MovementParameters.DEFAULT);
    }

    static Vector3d selectForcedLandingTarget(World world, PhysicsState state,
            int fireworks, boolean attached, MovementParameters parameters) {
        Objects.requireNonNull(world, "world");
        Objects.requireNonNull(state, "state");
        if (!state.position().isFinite() || !state.velocity().isFinite() || fireworks < 0)
            throw new IllegalArgumentException("Flight state must be finite and fireworks nonnegative");
        int[] flights = new int[fireworks];
        java.util.Arrays.fill(flights, 1);
        // Compatibility one-shot API. Live movements use the budgeted cursor overload.
        Vector3d target = new LandingSearch().select(world, state, flights, attached, parameters, Integer.MAX_VALUE);
        if (target != null) return target;
        throw new IllegalStateException("No known solid emergency landing ground in loaded terrain or within glide range");
    }

    static float[] landingPitches() { return PITCHES.clone(); }

    /** Shared verified standable column for selection and flight-only memory observations. */
    static Vector3d landingSite(World world, double x, double z, double ceiling) {
        // Require the complete support footprint to be loaded, not just its centre.
        for (int cx = (int) Math.floor(x - 2.1) >> 4; cx <= (int) Math.floor(x + 2.1) >> 4; cx++)
            for (int cz = (int) Math.floor(z - 2.1) >> 4; cz <= (int) Math.floor(z + 2.1) >> 4; cz++)
                if (!world.chunkLoaded(cx, cz)) return null;
        OptionalDouble top = world.findHighestCollisionTop(x - HALF_WIDTH, World.getMinWorldY(),
                z - HALF_WIDTH, x + HALF_WIDTH, ceiling, z + HALF_WIDTH);
        if (top.isEmpty()) return null;
        double y = top.getAsDouble();
        if (y + HEIGHT > World.getMaxWorldY() || !supportedLandingArea(world, x, y, z)
                || world.isBoxColliding(x - HALF_WIDTH, y, z - HALF_WIDTH, x + HALF_WIDTH, y + HEIGHT, z + HALF_WIDTH))
            return null;
        return new Vector3d(x, y, z);
    }

    static boolean supportedLandingArea(World world, double x, double y, double z) {
        for (int dx = -2; dx <= 2; dx++) {
            for (int dz = -2; dz <= 2; dz++) {
                var top = world.findHighestCollisionTop(x + dx - .1, World.getMinWorldY(), z + dz - .1,
                        x + dx + .1, y + .5, z + dz + .1);
                if (top.isEmpty() || Math.abs(top.getAsDouble() - y) > 1) return false;
            }
        }
        return true;
    }

    static double horizontalDistance(Vector3d first, Vector3d second) {
        return Math.hypot(first.x - second.x, first.z - second.z);
    }

    static float normalize(float angle) {
        float wrapped = angle % 360;
        if (wrapped >= 180) return wrapped - 360;
        if (wrapped < -180) return wrapped + 360;
        return wrapped;
    }

    @Override public FlightTrajectory predict(World world, PhysicsState initial, float yaw, float pitch,
            boolean boosted, int horizon, Vector3d target, boolean emergencyApproach) {
        var physics = stepper(yaw, pitch, boosted);
        return FlightTrajectory.simulate(world, initial, physics, horizon, (state, result) -> {
            Vector3d previous = state.position(), position = result.position();
            for (int x = (int) Math.floor(Math.min(previous.x, position.x) - HALF_WIDTH) >> 4;
                    x <= (int) Math.floor(Math.max(previous.x, position.x) + HALF_WIDTH) >> 4; x++) {
                for (int z = (int) Math.floor(Math.min(previous.z, position.z) - HALF_WIDTH) >> 4;
                        z <= (int) Math.floor(Math.max(previous.z, position.z) + HALF_WIDTH) >> 4; z++) {
                    if (!world.chunkLoaded(x, z)) return true;
                }
            }
            if (result.collidedX() || result.collidedZ()) return true;
            if (world.isBoxColliding(Math.min(previous.x, position.x) - HALF_WIDTH,
                    Math.min(previous.y, position.y), Math.min(previous.z, position.z) - HALF_WIDTH,
                    Math.max(previous.x, position.x) + HALF_WIDTH,
                    Math.max(previous.y, position.y) + HEIGHT,
                    Math.max(previous.z, position.z) + HALF_WIDTH)) return true;
            if (result.onGround()) return true;
            if (emergencyApproach && !boosted && position.y < target.y + horizontalDistance(position, target) / 5
                    && !supportedLandingArea(world, position.x, target.y, position.z)) return true;
            return position.y < World.getMinWorldY() || position.y + HEIGHT > World.getMaxWorldY();
        });
    }
}
