package xin.bbtt.physics;

import org.joml.Vector3d;
import xin.bbtt.world.World;

/** Bounded emergency walking proof, independent of the abandoned navigation goal.
 * Initial damaging contact is allowed only as a finite prefix; after escaping,
 * every remaining step (including braking to rest) must be hazard free.
 */
public final class GroundHazardExit {
    private GroundHazardExit() {}
    public static final PlayerInput BRAKE = new PlayerInput(false, false, false, false, false, true, false);
    private static final PlayerInput WALK = new PlayerInput(true, false, false, false, false, false, false);
    public record Command(float yaw, PlayerInput input) {}

    /** Detect a known damaging coast trajectory, including drift after nongliding touchdown.
     * Unknown terrain alone and residual burning metadata do not justify a new owner.
     */
    public static boolean airborneRisk(World world, PhysicsState initial, MovementParameters parameters) {
        if (!initial.position().isFinite() || !initial.velocity().isFinite()
                || !Double.isFinite(initial.fallDistance())) return false;
        boolean[] risk = {false};
        world.readTerrain(() -> {
            PhysicsState state = initial;
            for (int tick = 0; tick < 60; tick++) {
                if (Thread.currentThread().isInterrupted()) return false;
                var next = new WalkingPhysics(PlayerInput.NONE, 0, true, parameters)
                        .step(world, state, PlayerPose.HALF_WIDTH, (double) 1.8f);
                if (!FlightSafety.knownStep(world, state, next)) return false;
                if (FlightHazards.damage(world, state, next, DamageParameters.NONE, true).raw() > 0
                        || FlightHazards.damage(world, state, next, DamageParameters.NONE, false).raw() > 0) {
                    risk[0] = true;
                    return true;
                }
                if (next.onGround() && next.velocity().x == 0 && next.velocity().z == 0) return true;
                state = FlightSafety.nextState(next);
            }
            return true;
        });
        return risk[0];
    }

    public static Command prove(World world, PhysicsState initial, MovementParameters parameters) {
        if (!initial.position().isFinite() || !initial.velocity().isFinite()
                || !Double.isFinite(initial.fallDistance())) return null;
        Command[] best = {null};
        world.readTerrain(() -> {
            int bestExposure = Integer.MAX_VALUE;
            for (int direction = 0; direction < 16; direction++) {
                for (int duration : new int[] {0, 6, 12, 24, 40}) {
                    if (Thread.currentThread().isInterrupted()) return false;
                    float yaw = direction * 22.5f;
                    int exposure = simulate(world, initial, parameters, yaw, duration);
                    if (exposure >= 0 && exposure < bestExposure) {
                        bestExposure = exposure;
                        best[0] = new Command(yaw, duration == 0 ? BRAKE : WALK);
                        if (exposure == 0) return true;
                    }
                }
            }
            return true;
        });
        return best[0];
    }

    private static int simulate(World world, PhysicsState initial, MovementParameters parameters,
                                float yaw, int duration) {
        PhysicsState state = initial;
        boolean escaped = FlightHazards.safeLanding(world, initial.position());
        int exposure = 0;
        for (int tick = 0; tick < duration + 80; tick++) {
            if (Thread.currentThread().isInterrupted()) return -1;
            PhysicsResult next = new WalkingPhysics(tick < duration ? WALK : BRAKE, yaw, true, parameters)
                    .step(world, state, PlayerPose.HALF_WIDTH, (double) 1.8f);
            if (!FlightSafety.knownStep(world, state, next)
                    || next.position().y + (double) 1.8f > World.getMaxWorldY()) return -1;
            boolean contact = FlightHazards.damage(world, state, next, DamageParameters.NONE, true).raw() > 0
                    || FlightHazards.damage(world, state, next, DamageParameters.NONE, false).raw() > 0;
            if (escaped && contact) return -1;
            if (contact && ++exposure > 24) return -1;
            if (FlightHazards.safeLanding(world, next.position())) escaped = true;
            if (next.onGround() && state.fallDistance()
                    + Math.max(0, state.position().y - next.position().y) > 3) return -1;
            Vector3d p = next.position();
            if (tick >= duration && escaped && next.onGround() && next.velocity().x == 0
                    && next.velocity().z == 0 && next.velocity().y <= 0
                    && world.isBoxColliding(p.x - PlayerPose.HALF_WIDTH, p.y - 1.0e-4,
                    p.z - PlayerPose.HALF_WIDTH, p.x + PlayerPose.HALF_WIDTH, p.y,
                    p.z + PlayerPose.HALF_WIDTH)
                    && !world.isBoxColliding(p.x - PlayerPose.HALF_WIDTH, p.y, p.z - PlayerPose.HALF_WIDTH,
                    p.x + PlayerPose.HALF_WIDTH, p.y + (double) 1.8f, p.z + PlayerPose.HALF_WIDTH)
                    && EnvironmentalPhysics.sample(world, p, PlayerPose.HALF_WIDTH, (double) 1.8f)
                    .waterHeight() == 0) return exposure;
            state = FlightSafety.nextState(next);
        }
        return -1;
    }
}
