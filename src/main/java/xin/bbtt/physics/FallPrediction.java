package xin.bbtt.physics;

import org.joml.Vector3d;
import xin.bbtt.world.World;

/** Read-only, no-input forecast using the same walking stepper as execution. */
public final class FallPrediction {
    private static final double HALF_WIDTH = (double) 0.6f / 2;
    private static final double HEIGHT = (double) 1.8f;
    // A simulation work limit, not a deployment height or damage threshold.
    private static final int MAX_TICKS = 2400;

    private FallPrediction() {}

    public record Result(boolean dangerous, double fallDistance, int damage) {}

    public static Result predict(World world, PhysicsState initial, MovementParameters parameters) {
        PhysicsState state = initial;
        PhysicsStepper stepper = new WalkingPhysics(PlayerInput.NONE, 0, false, parameters);
        if (state.onGround()) return new Result(false, 0, 0);
        for (int tick = 0; tick < MAX_TICKS; tick++) {
            Vector3d position = state.position();
            if (position.y < World.getMinWorldY() || !loaded(world, position)) {
                return new Result(true, state.fallDistance(), 0);
            }
            var environment = EnvironmentalPhysics.sample(world, position, HALF_WIDTH, HEIGHT);
            if (environment.waterHeight() > 0) return new Result(false, 0, 0);
            PhysicsResult next = stepper.step(world, state, HALF_WIDTH, HEIGHT);
            if (!loaded(world, next.position())) return new Result(true, state.fallDistance(), 0);
            if (next.onGround()) {
                double distance = parameters.slowFalling() || parameters.levitationAmplifier() >= 0
                        ? 0 : state.fallDistance() + Math.max(0, position.y - next.position().y);
                Vector3d support = next.position().add(0, -(double) 0.2f, 0);
                String block = world.getBlockStateAt(support).blockName().replace("minecraft:", "");
                double multiplier = switch (block) {
                    case "slime_block", "powder_snow" -> 0;
                    case "hay_block", "honey_block" -> (double) 0.2f;
                    default -> 1;
                };
                if (block.endsWith("_bed")) distance *= 0.5;
                int damage = damage(distance, multiplier, parameters);
                return new Result(damage > 0, distance, damage);
            }
            state = new PhysicsState(next.position(), next.velocity(), false, next.fallDistance(), next.pose());
        }
        return new Result(true, state.fallDistance(), 0);
    }

    /** 1.21.11 LivingEntity.computeFallDamage; jump boost is a safe-fall attribute modifier. */
    public static int damage(double distance, double blockMultiplier, MovementParameters parameters) {
        return DamageModel.rawFall(distance, blockMultiplier, parameters);
    }

    private static boolean loaded(World world, Vector3d p) {
        for (int x = (int) Math.floor(p.x - HALF_WIDTH) >> 4; x <= (int) Math.floor(p.x + HALF_WIDTH) >> 4; x++) {
            for (int z = (int) Math.floor(p.z - HALF_WIDTH) >> 4; z <= (int) Math.floor(p.z + HALF_WIDTH) >> 4; z++) {
                if (!world.chunkLoaded(x, z)) return false;
            }
        }
        return true;
    }
}
