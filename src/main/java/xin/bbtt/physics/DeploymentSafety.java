package xin.bbtt.physics;

import org.joml.Vector3d;
import xin.bbtt.world.World;

/** Physical abort proof for a stationary launch; a start packet is not server acceptance. */
public final class DeploymentSafety {
    private DeploymentSafety() {}

    public record Assessment(boolean safe, String reason) {}

    /** No deployment, or interruption during the first five glide ticks, must land safely. */
    public static Assessment assess(World world, PhysicsState initial, MovementParameters parameters,
                                    float yaw, float pitch) {
        return assess(world, initial, parameters, yaw, pitch, false);
    }

    /**
     * Relocation candidates require the full interruption and independent server/client
     * landing proof for metadata delays 1, 2 and 4, even on broad inland support.
     * Like assess, this fails closed as soon as any required branch is unsafe.
     */
    public static Assessment assessRelocation(World world, PhysicsState initial, MovementParameters parameters,
                                              float yaw, float pitch) {
        return assess(world, initial, parameters, yaw, pitch, true);
    }

    private static Assessment assess(World world, PhysicsState initial, MovementParameters parameters,
                                     float yaw, float pitch, boolean requireLanding) {
        PhysicsState state = initial;
        if (!returnsToSupport(world, state, parameters, yaw, initial.onGround()))
            return new Assessment(false, "no-deploy branch cannot return to safe support");
        for (int tick = 0; tick < 40; tick++) {
            if (Thread.currentThread().isInterrupted()) return new Assessment(false, "interrupted");
            if (!state.onGround() && state.velocity().y <= 0) {
                for (int glide = 0; glide < 5; glide++) {
                    PhysicsResult next = new GlidingPhysics(yaw, pitch, false, PlayerInput.NONE, parameters)
                            .step(world, state, PlayerPose.HALF_WIDTH, glide == 0 ? (double) 1.8f : (double) 0.6f);
                    if (!safeSegment(world, state.position(), next.position()) || next.onGround())
                        return new Assessment(false, "deployment trajectory blocked");
                    state = state(next);
                    if (!returnsToSupport(world, state, parameters, yaw, false))
                        return new Assessment(false, "interrupted glide tick " + (glide + 1)
                                + " cannot return to safe support");
                }
                // A five-tick clear corridor says nothing about the first touchdown.
                // Continue the fixed launch controls to contact, then model independent
                // server/client glide flags through the complete ground coast.
                if (requireLanding || hazardNearby(world, initial.position())) {
                    for (int delay : new int[]{1, 2, 4}) {
                        Assessment landing = assessLanding(world, state, parameters, yaw, pitch, delay);
                        if (!landing.safe()) return landing;
                    }
                    return new Assessment(true, "bounded shoreline launch and delayed-metadata landing branches stop safely");
                }
                return new Assessment(true, "bounded launch and early interruption branches return to safe support");
            }
            PhysicsResult next = new WalkingPhysics(PlayerInput.NONE.withJump(tick == 0 && initial.onGround()),
                    yaw, true, parameters).step(world, state, PlayerPose.HALF_WIDTH, (double) 1.8f);
            if (!safeSegment(world, state.position(), next.position()) || next.onGround())
                return new Assessment(false, "jump trajectory blocked");
            state = state(next);
        }
        return new Assessment(false, "no descending deployment within horizon");
    }

    /**
     * Local shoreline gate: nearby damaging/fluid contact or a downward support
     * edge needs the stricter independent-metadata landing proof.
     * Broad inland terrain retains the early-interruption deployment contract.
     */
    private static boolean hazardNearby(World world, Vector3d feet) {
        // assess is also called at the descending deployment tick: compare nearby
        // support against the launch surface, not against airborne feet height.
        var support = world.findHighestCollisionTop(feet.x - PlayerPose.HALF_WIDTH, feet.y - 3,
                feet.z - PlayerPose.HALF_WIDTH, feet.x + PlayerPose.HALF_WIDTH, feet.y + 1.0e-4,
                feet.z + PlayerPose.HALF_WIDTH);
        if (support.isEmpty()) return true;
        feet = new Vector3d(feet.x, support.getAsDouble(), feet.z);
        for (int dx = -2; dx <= 2; dx++) for (int dz = -2; dz <= 2; dz++) {
            Vector3d p = new Vector3d(feet).add(dx, 0, dz);
            if (!world.chunkLoaded((int) Math.floor(p.x) >> 4, (int) Math.floor(p.z) >> 4)) return true;
            if (!world.isBoxColliding(p.x - PlayerPose.HALF_WIDTH, p.y - 1.0e-4,
                    p.z - PlayerPose.HALF_WIDTH, p.x + PlayerPose.HALF_WIDTH, p.y,
                    p.z + PlayerPose.HALF_WIDTH)) return true;
            for (int dy = 0; dy <= 3; dy++) {
                Vector3d sample = new Vector3d(p).add(0, -dy, 0);
                if (!FlightHazards.safeLanding(world, sample)) return true;
                var environment = EnvironmentalPhysics.sample(world, sample, PlayerPose.HALF_WIDTH, (double) 1.8f);
                if (environment.waterHeight() > 0 || environment.lavaHeight() > 0) return true;
            }
        }
        return false;
    }

    /**
     * Fixed launch controls until first contact, then reverse-yaw sneak braking.
     * LivingEntity.tickMovement calls server-only tickGliding BEFORE travel; once
     * contact is present at tick start the server clears glide and walks that tick.
     * A client can still glide until metadata arrives. Both paths must stop safely.
     * This bounded proof covers these controls and delays, not arbitrary future input.
     * It is a rejection gate, not proof of arbitrary controller execution or Grim
     * acceptance. In particular, even zero horizontal contact speed can diverge:
     * pitch-zero glide creates horizontal lift while server walking does not.
     */
    public static Assessment assessLanding(World world, PhysicsState initial, MovementParameters parameters,
                                            float yaw, float pitch, int metadataDelay) {
        if (metadataDelay < 1 || metadataDelay > 4)
            return new Assessment(false, "metadata delay outside proof horizon");
        PhysicsState contact = initial;
        for (int tick = 0; !contact.onGround(); tick++) {
            if (tick >= 80 || Thread.currentThread().isInterrupted())
                return new Assessment(false, "landing contact outside proof horizon");
            PhysicsResult next = new GlidingPhysics(yaw, pitch, false, PlayerInput.NONE, parameters)
                    .step(world, contact, PlayerPose.HALF_WIDTH, contact.pose().height());
            if (!safeSegment(world, contact.position(), next.position()))
                return new Assessment(false, "landing trajectory touches hazard or unknown terrain");
            contact = state(next);
            if (!returnsToSupport(world, contact, parameters, yaw, false))
                return new Assessment(false, "interrupted glide before landing cannot stop safely");
        }
        PhysicsState server = contact;
        PhysicsState client = contact;
        PlayerInput hold = new PlayerInput(false, false, false, false, false, true, false);
        for (int tick = 0; tick < 80; tick++) {
            if (Thread.currentThread().isInterrupted()) return new Assessment(false, "interrupted");
            Vector3d velocity = client.velocity();
            float reverse = (float) Math.toDegrees(Math.atan2(velocity.x, -velocity.z));
            PhysicsResult serverNext = new WalkingPhysics(hold, reverse, true, parameters)
                    .step(world, server, PlayerPose.HALF_WIDTH, server.pose().height());
            PhysicsResult clientNext = tick < metadataDelay
                    ? new GlidingPhysics(reverse, 0, false, hold, parameters)
                        .step(world, client, PlayerPose.HALF_WIDTH, client.pose().height())
                    : new WalkingPhysics(hold, reverse, true, parameters)
                        .step(world, client, PlayerPose.HALF_WIDTH, client.pose().height());
            if (!safeSegment(world, server.position(), serverNext.position())
                    || !safeSegment(world, client.position(), clientNext.position())
                    || !serverNext.onGround() || !clientNext.onGround())
                return new Assessment(false, "delayed metadata landing loses safe support");
            server = state(serverNext);
            client = state(clientNext);
            // Each side must stop safely; metadata latency does not require equal positions.
            if (tick >= metadataDelay && stopped(server) && stopped(client))
                return new Assessment(true, "both landing branches stop on safe support");
        }
        return new Assessment(false, "delayed metadata landing does not stop within horizon");
    }

    private static boolean stopped(PhysicsState state) {
        return state.onGround() && state.velocity().x == 0 && state.velocity().z == 0;
    }

    /**
     * Prove one actual walking command followed by interruption (neutral input).
     * Ground contact alone is not enough: residual horizontal motion can still
     * carry the player off a shore on a later tick. Fail closed after 80 ticks.
     */
    public static Assessment assessWalking(World world, PhysicsState initial, MovementParameters parameters,
                                            float yaw, PlayerInput input) {
        PhysicsState state = initial;
        for (int tick = 0; tick < 80; tick++) {
            if (Thread.currentThread().isInterrupted()) return new Assessment(false, "interrupted");
            PhysicsResult next = new WalkingPhysics(tick == 0 ? input : PlayerInput.NONE, yaw, true, parameters)
                    .step(world, state, PlayerPose.HALF_WIDTH, state.pose().height());
            if (!safeSegment(world, state.position(), next.position()))
                return new Assessment(false, "walking coast touches hazard or unknown terrain");
            if (next.onGround()) {
                Vector3d p = next.position();
                if (state.fallDistance() + Math.max(0, state.position().y - p.y) > 3)
                    return new Assessment(false, "walking coast causes fall damage");
                if (next.velocity().x == 0 && next.velocity().z == 0
                        && world.isBoxColliding(p.x - PlayerPose.HALF_WIDTH, p.y - 1.0e-4,
                        p.z - PlayerPose.HALF_WIDTH, p.x + PlayerPose.HALF_WIDTH, p.y,
                        p.z + PlayerPose.HALF_WIDTH))
                    return new Assessment(true, "walking coast stops on safe support");
            }
            state = state(next);
        }
        return new Assessment(false, "walking coast does not settle within horizon");
    }

    private static boolean returnsToSupport(World world, PhysicsState initial, MovementParameters parameters,
                                            float yaw, boolean jump) {
        PhysicsState state = initial;
        for (int tick = 0; tick < 80; tick++) {
            if (Thread.currentThread().isInterrupted()) return false;
            PhysicsResult next = new WalkingPhysics(PlayerInput.NONE.withJump(jump && tick == 0), yaw, true, parameters)
                    .step(world, state, PlayerPose.HALF_WIDTH, (double) 1.8f);
            if (!safeSegment(world, state.position(), next.position())) return false;
            if (next.onGround()) {
                Vector3d p = next.position();
                if (state.fallDistance() + Math.max(0, state.position().y - p.y) > 3) return false;
                if (next.velocity().x == 0 && next.velocity().z == 0
                        && world.isBoxColliding(p.x - PlayerPose.HALF_WIDTH, p.y - 1.0e-4,
                        p.z - PlayerPose.HALF_WIDTH, p.x + PlayerPose.HALF_WIDTH, p.y,
                        p.z + PlayerPose.HALF_WIDTH)
                        && !world.isBoxColliding(p.x - PlayerPose.HALF_WIDTH, p.y,
                        p.z - PlayerPose.HALF_WIDTH, p.x + PlayerPose.HALF_WIDTH, p.y + (double) 1.8f,
                        p.z + PlayerPose.HALF_WIDTH)) return true;
                // Touchdown is not terminal: prove the complete neutral-input coast.
            }
            state = state(next);
        }
        return false;
    }

    private static boolean safeSegment(World world, Vector3d from, Vector3d to) {
        int steps = Math.max(1, (int) Math.ceil(from.distance(to) / .1));
        for (int i = 0; i <= steps; i++) {
            Vector3d p = new Vector3d(from).lerp(to, (double) i / steps);
            for (int x = (int) Math.floor(p.x - PlayerPose.HALF_WIDTH) >> 4;
                    x <= (int) Math.floor(p.x + PlayerPose.HALF_WIDTH) >> 4; x++)
                for (int z = (int) Math.floor(p.z - PlayerPose.HALF_WIDTH) >> 4;
                        z <= (int) Math.floor(p.z + PlayerPose.HALF_WIDTH) >> 4; z++)
                    if (!world.chunkLoaded(x, z)) return false;
            if (!FlightHazards.safeLanding(world, p)) return false;
            var environment = EnvironmentalPhysics.sample(world, p, PlayerPose.HALF_WIDTH, (double) 1.8f);
            if (environment.waterHeight() > 0 || environment.lavaHeight() > 0) return false;
        }
        return true;
    }

    private static PhysicsState state(PhysicsResult result) {
        return new PhysicsState(result.position(), result.velocity(), result.onGround(), result.fallDistance(), result.pose());
    }
}
