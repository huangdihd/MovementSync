package xin.bbtt.physics;

import java.util.List;
import org.joml.Vector3d;
import xin.bbtt.world.World;

/** Damage-first classification of bounded flight trajectories. */
final class FlightSafety {
    private static final int HORIZON = 24;
    static final double HALF_WIDTH = 0.299;
    static final double HEIGHT = 0.6;
    // Entity.limitFallDistance (Yarn 1.21.11): strictly greater than -0.5.
    static final double FALL_RESET_SPEED = -0.5;
    private final MovementParameters movement;
    private final DamageParameters mitigation;
    private final double health;
    private final RecoveryStrategy recovery;
    private final List<DamageRule> rules = List.of(this::wallDamage, this::fallDamage);

    FlightSafety(MovementParameters movement, DamageParameters mitigation, double health) {
        this(movement, mitigation, health, new float[] {-55, -30, -10, 0, 10});
    }

    FlightSafety(MovementParameters movement, DamageParameters mitigation, double health, float[] pitches) {
        this.movement = movement;
        this.mitigation = mitigation;
        this.health = health;
        this.recovery = new VanillaGlideRecovery(this, pitches);
    }

    record EmergencyCost(int damageClass, double damage, int approachClass, double score)
            implements Comparable<EmergencyCost> {
        @Override public int compareTo(EmergencyCost other) {
            int order = Integer.compare(damageClass, other.damageClass);
            if (order == 0) order = Double.compare(damage, other.damage);
            if (order == 0) order = Integer.compare(approachClass, other.approachClass);
            return order == 0 ? Double.compare(score, other.score) : order;
        }
    }

    record Impact(boolean unknown, float raw, float reduced) {
        static final Impact UNKNOWN = new Impact(true, 0, 0);
        static final Impact SAFE = new Impact(false, 0, 0);
        int rank(double health) { return unknown ? 3 : raw == 0 ? 0 : reduced > 0 && reduced >= health ? 2 : 1; }
    }

    interface RecoveryStrategy {
        Impact recover(World world, PhysicsState state, float yaw, boolean attached);
    }

    int compareImpact(Impact a, Impact b) {
        int rank = Integer.compare(a.rank(health), b.rank(health));
        return rank == 0 ? Float.compare(a.reduced(), b.reduced()) : rank;
    }

    PhysicsStepper stepper(float yaw, float pitch, boolean attached) {
        return new GlidingPhysics(yaw, pitch, attached, movement.gravity(), movement.slowFalling());
    }

    double emergencyDamage(World world, PhysicsState initial, float yaw, float pitch, boolean attached) {
        Impact impact = predictEmergencyImpact(world, initial, yaw, pitch, attached);
        return impact.unknown() ? Double.POSITIVE_INFINITY : impact.reduced();
    }

    Impact predictEmergencyImpact(World world, PhysicsState initial, float yaw, float pitch, boolean attached) {
        return predictEmergencyImpact(world, initial, yaw, pitch, attached,
                stepper(yaw, pitch, attached), recovery);
    }

    Impact predictEmergencyImpact(World world, PhysicsState initial, float yaw, float pitch, boolean attached,
            PhysicsStepper physics, RecoveryStrategy recovery) {
        CheckedTrajectory checked = simulate(world, initial, physics, HORIZON, yaw, pitch, attached, false);
        if (checked.terminalImpact() != null) return checked.terminalImpact();
        PhysicsState state = checked.finalState();
        if (state.velocity().y > FALL_RESET_SPEED) return Impact.SAFE;
        return recovery.recover(world, state, yaw, attached);
    }

    // A null terminal impact means the horizon was exhausted without a classification.
    record CheckedTrajectory(FlightTrajectory trajectory, Impact terminalImpact) {
        PhysicsState finalState() {
            return trajectory.size() == 0 ? trajectory.initial() : nextState(trajectory.result(trajectory.size() - 1));
        }
    }

    CheckedTrajectory simulate(World world, PhysicsState initial, PhysicsStepper physics, int horizon,
            float yaw, float pitch, boolean attached, boolean stopOnRecovery) {
        Impact[] terminal = {null};
        FlightTrajectory trajectory = FlightTrajectory.simulate(world, initial, physics, horizon, (state, result) -> {
            if (!knownStep(world, state, result)) {
                terminal[0] = Impact.UNKNOWN;
                return true;
            }
            var lava = FlightHazards.damage(world, state, result, mitigation, true);
            var fire = FlightHazards.damage(world, state, result, mitigation, false);
            if (lava.raw() > 0 || fire.raw() > 0) {
                terminal[0] = Impact.UNKNOWN; // Forbidden cells even during emergency ranking.
                return true;
            }
            if (collision(result)) {
                Impact impact = collisionDamage(world, state, result, yaw, pitch, attached);
                if (result.onGround() || impact.raw() > 0) {
                    terminal[0] = impact;
                    return true;
                }
                // A harmless brush does not reset accumulated fall distance.
            }
            if (stopOnRecovery && result.velocity().y > FALL_RESET_SPEED) {
                terminal[0] = Impact.SAFE;
                return true;
            }
            return false;
        });
        return new CheckedTrajectory(trajectory, terminal[0]);
    }

    static PhysicsState nextState(PhysicsResult result) {
        return new PhysicsState(result.position(), result.velocity(), result.onGround(), result.fallDistance());
    }

    static boolean collision(PhysicsResult result) {
        return result.onGround() || result.collidedX() || result.collidedZ();
    }

    static boolean knownStep(World world, PhysicsState state, PhysicsResult result) {
        Vector3d from = state.position(), to = result.position();
        if (to.y < World.getMinWorldY() || to.y + HEIGHT > World.getMaxWorldY()) return false;
        for (int x = (int) Math.floor(Math.min(from.x, to.x) - HALF_WIDTH) >> 4;
                x <= (int) Math.floor(Math.max(from.x, to.x) + HALF_WIDTH) >> 4; x++) {
            for (int z = (int) Math.floor(Math.min(from.z, to.z) - HALF_WIDTH) >> 4;
                    z <= (int) Math.floor(Math.max(from.z, to.z) + HALF_WIDTH) >> 4; z++) {
                if (!world.chunkLoaded(x, z)) return false;
            }
        }
        return true;
    }

    private interface DamageRule {
        DamageModel.Damage damage(World world, PhysicsState state, PhysicsResult result,
                float yaw, float pitch, boolean attached);
    }

    Impact collisionDamage(World world, PhysicsState state, PhysicsResult result,
            float yaw, float pitch, boolean attached) {
        DamageModel.Damage total = null;
        for (DamageRule rule : rules) {
            DamageModel.Damage damage = rule.damage(world, state, result, yaw, pitch, attached);
            // Seed with the first rule to preserve signed zero and float addition order.
            total = total == null ? damage
                    : new DamageModel.Damage(total.raw() + damage.raw(), total.reduced() + damage.reduced());
        }
        return total == null ? Impact.SAFE : new Impact(false, total.raw(), total.reduced());
    }

    private DamageModel.Damage wallDamage(World world, PhysicsState state, PhysicsResult result,
            float yaw, float pitch, boolean attached) {
        // Rocket acceleration happens after travel; wall damage uses pre-rocket clipped motion.
        PhysicsResult clipped = attached
                ? stepper(yaw, pitch, false).step(world, state, HALF_WIDTH, HEIGHT)
                : result;
        double speed = Math.hypot(state.velocity().x, state.velocity().z);
        return DamageModel.wall(result.collidedX() || result.collidedZ(),
                speed < .003 ? 0 : speed,
                Math.hypot(clipped.velocity().x, clipped.velocity().z), mitigation);
    }

    private DamageModel.Damage fallDamage(World world, PhysicsState state, PhysicsResult result,
            float yaw, float pitch, boolean attached) {
        double distance = state.velocity().y > FALL_RESET_SPEED ? Math.min(1, state.fallDistance()) : state.fallDistance();
        distance += (float) Math.max(0, state.position().y - result.position().y);
        String block = world.getBlockStateAt(result.position().add(0, -(double) .2f, 0))
                .blockName().replace("minecraft:", "");
        double multiplier = switch (block) {
            case "slime_block", "powder_snow" -> 0;
            case "hay_block", "honey_block" -> (double) .2f;
            default -> 1;
        };
        if (block.endsWith("_bed")) distance *= .5;
        return DamageModel.fall(result.onGround() && !movement.slowFalling() ? distance : 0,
                multiplier, movement, mitigation);
    }
}
