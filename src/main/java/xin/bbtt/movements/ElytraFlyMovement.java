package xin.bbtt.movements;

import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.movement.Movement;
import xin.bbtt.physics.ElytraFlightController;
import xin.bbtt.physics.PhysicsState;

/** Owns flight requests and lifecycle; steering predictions never mutate live physics. */
public final class ElytraFlyMovement extends Movement implements NavigationBoundMovement {
    public static final double DEFAULT_ARRIVAL_RADIUS = 16;
    private final double arrivalRadius;
    private final Vector3d target;
    private long generation;
    private int takeoffTicks;
    private int flightTicks;
    private int rocketCooldown;
    private boolean airborne;

    public ElytraFlyMovement(Vector3d target) {
        this(target, MovementSync.INSTANCE.getNavigationGeneration());
    }

    public ElytraFlyMovement(Vector3d target, long generation) {
        this(target, generation, DEFAULT_ARRIVAL_RADIUS);
    }

    public ElytraFlyMovement(Vector3d target, long generation, double arrivalRadius) {
        if (!Double.isFinite(arrivalRadius) || arrivalRadius <= 0) throw new IllegalArgumentException("Radius must be positive");
        this.arrivalRadius = arrivalRadius;
        if (target == null || !target.isFinite()) throw new IllegalArgumentException("Target must be finite");
        this.target = new Vector3d(target);
        this.generation = generation;
    }

    @Override
    public void bindNavigationRequest(long generation) {
        this.generation = generation;
    }

    public Vector3d getTarget() {
        return new Vector3d(target);
    }

    @Override
    public void init() {
        MovementSync movementSync = MovementSync.INSTANCE;
        if (!movementSync.runIfNavigationRequestCurrent(generation, () -> {
            if (isFinished()) return;
            // Remember an inherited glide before physics can land it between action ticks.
            airborne = movementSync.isGliding();
            if (!airborne && movementSync.onGround.get()) movementSync.jump();
        })) setFinished(true);
    }

    @Override
    public void onTick() {
        if (isFinished()) return;
        MovementSync movementSync = MovementSync.INSTANCE;
        if (!movementSync.isNavigationRequestCurrent(generation)) {
            setFinished(true);
            return;
        }
        if (!airborne && !movementSync.isGliding()) {
            movementSync.runIfNavigationRequestCurrent(generation, () -> {
                if (isFinished()) return;
                if (++takeoffTicks > 60 || movementSync.isRiding()) {
                    finish(false);
                    return;
                }
                if (!movementSync.onGround.get() && movementSync.velocity.get().y <= 0) {
                    airborne = movementSync.startGliding();
                }
            });
            return;
        }
        airborne = true;
        PhysicsState state = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(), movementSync.onGround.get(), movementSync.fallDistance.get());
        if (state.onGround() || !movementSync.isGliding() || ++flightTicks > 12000) {
            movementSync.runIfNavigationRequestCurrent(generation, () -> {
                if (isFinished()) return;
                Vector3d position = state.position();
                finish(state.onGround() && Math.hypot(position.x - target.x, position.z - target.z) <= arrivalRadius);
            });
            return;
        }
        if (rocketCooldown > 0) rocketCooldown--;
        ElytraFlightController.Decision decision;
        try {
            xin.bbtt.physics.MovementParameters parameters = movementSync.getMovementParameters();
            ElytraFlightController controller = new ElytraFlightController(parameters.gravity(), parameters.slowFalling());
            decision = controller.decide(movementSync.getWorld(), state,
                    movementSync.yaw.get(), movementSync.pitch.get(),
                    movementSync.hasAttachedFirework() || rocketCooldown > 35,
                    target, rocketCooldown == 0 ? movementSync.countHotbarFireworks() : 0, arrivalRadius);
        } catch (IllegalStateException error) {
            MovementSync.getLogger().warn("Flight prediction failed at {} with velocity {}: {}",
                    state.position(), state.velocity(), error.getMessage());
            movementSync.runIfNavigationRequestCurrent(generation, () -> finish(false));
            return;
        }
        // Cancellation may happen while trajectory evaluation is running. Guard the actual writes too.
        movementSync.runIfNavigationRequestCurrent(generation, () -> {
            if (isFinished()) return;
            movementSync.setYawToward(decision.yaw());
            movementSync.pitch.set(decision.pitch());
            if (decision.firework() && !movementSync.hasAttachedFirework()) {
                rocketCooldown = 40;
                movementSync.useFirework();
            }
        });
    }

    private void finish(boolean arrived) {
        setFinished(true);
        MovementSync.getLogger().info(LangManager.get(arrived
                ? "movementsync.command.elytra.fly.arrived" : "movementsync.command.elytra.fly.failed"));
        if (!arrived) {
            Vector3d position = MovementSync.INSTANCE.position.get();
            MovementSync.getLogger().info("Flight failed: remaining horizontal distance={} blocks, arrival radius={} blocks",
                    Math.hypot(position.x - target.x, position.z - target.z), arrivalRadius);
        }
    }

    @Override
    public long getTime() {
        return -1;
    }

    @Override
    public void onStop() {
        // Cancelling steering must not fabricate a landing or switch off airborne physics.
        setFinished(true);
    }
}
