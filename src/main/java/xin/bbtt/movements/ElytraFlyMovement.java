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
    private boolean emergencyLanding;
    private boolean safeLandingLeg;
    private boolean emergencyHoldWarned;
    private boolean independentLanding;

    /** Safety control has no navigation request or destination intent. */
    public ElytraFlyMovement forIndependentLanding() {
        independentLanding = true;
        return forSafeLanding();
    }

    private boolean runIfCurrent(Runnable action) {
        MovementSync sync = MovementSync.INSTANCE;
        return independentLanding ? sync.runIfSafetyMovementCurrent(this, action)
                : sync.runIfNavigationRequestCurrent(generation, action);
    }

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

    /** Deploy through takeoff, then use the existing solid-ground emergency landing controller. */
    public ElytraFlyMovement forSafeLanding() {
        emergencyLanding = true;
        safeLandingLeg = true;
        return this;
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
        if (!runIfCurrent(() -> {
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
        if (!runIfCurrent(() -> {})) {
            setFinished(true);
            return;
        }
        if (!airborne && !movementSync.isGliding()) {
            runIfCurrent(() -> {
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
            runIfCurrent(() -> {
                if (isFinished()) return;
                Vector3d position = state.position();
                if (safeLandingLeg && state.onGround()) {
                    setFinished(true);
                    MovementSync.getLogger().info("Elytra safe landing complete at {}", position);
                } else {
                    finish(!emergencyLanding && state.onGround()
                            && Math.hypot(position.x - target.x, position.z - target.z) <= arrivalRadius);
                }
            });
            return;
        }
        if (rocketCooldown > 0) rocketCooldown--;
        var equipmentAction = movementSync.inventoryManager.tickElytraSwap(false,
                movementSync::queueBeforeMovement, () -> xin.bbtt.mcbot.Bot.INSTANCE.getAndIncreaseSequence(),
                movementSync.yaw.get(), movementSync.pitch.get());
        ElytraFlightController.Decision decision;
        try {
            xin.bbtt.physics.MovementParameters parameters = movementSync.getMovementParameters();
            ElytraFlightController controller = new ElytraFlightController(parameters.gravity(), parameters.slowFalling());
            if (!emergencyLanding && equipmentAction == xin.bbtt.inventory.InventoryManager.ElytraAction.LAND) {
                // Spare-equipment replacement must be attempted before latching this emergency.
                emergencyLanding = true;
                MovementSync.getLogger().warn("Elytra durability low; seeking solid emergency landing ground");
            }
            boolean attached = movementSync.hasAttachedFirework() || rocketCooldown > 35;
            int fireworks = rocketCooldown == 0 ? movementSync.countHotbarFireworks() : 0;
            if (!emergencyLanding && !attached && movementSync.countHotbarFireworks() == 0
                    && "minecraft:the_end".equals(movementSync.getRecoveryDimension())) {
                Vector3d p = state.position();
                var support = movementSync.getWorld().findHighestCollisionTop(p.x - .299,
                        xin.bbtt.world.World.getMinWorldY(), p.z - .299, p.x + .299, p.y, p.z + .299);
                if (support.isEmpty()) {
                    emergencyLanding = true;
                    MovementSync.getLogger().warn("Fireworks exhausted above void; diverting to known solid ground");
                }
            }
            ElytraFlightController.Decision selected;
            if (emergencyLanding) {
                selected = controller.forceLand(movementSync.getWorld(), state,
                        movementSync.yaw.get(), movementSync.pitch.get(), attached, fireworks, arrivalRadius);
            } else {
                try {
                    selected = controller.decide(movementSync.getWorld(), state,
                            movementSync.yaw.get(), movementSync.pitch.get(), attached, target, fireworks, arrivalRadius);
                } catch (IllegalStateException unavailable) {
                    emergencyLanding = true;
                    selected = controller.forceLand(movementSync.getWorld(), state,
                            movementSync.yaw.get(), movementSync.pitch.get(), attached, fireworks, arrivalRadius);
                }
                if (selected.phase() == ElytraFlightController.Phase.FORCED_LANDING) {
                    emergencyLanding = true;
                    selected = controller.forceLand(movementSync.getWorld(), state,
                            movementSync.yaw.get(), movementSync.pitch.get(), attached, fireworks, arrivalRadius);
                }
            }
            decision = selected;
        } catch (IllegalStateException error) {
            emergencyLanding = true;
            if (!emergencyHoldWarned) {
                MovementSync.getLogger().warn("Emergency landing unavailable at {} with velocity {}: {}; "
                                + "retaining unpowered steering and retrying; safe touchdown is not guaranteed",
                        state.position(), state.velocity(), error.getMessage());
                emergencyHoldWarned = true;
            }
            var parameters = movementSync.getMovementParameters();
            decision = new ElytraFlightController(parameters.gravity(), parameters.slowFalling())
                    .emergencyHold(movementSync.getWorld(), state, movementSync.yaw.get(), movementSync.pitch.get(),
                            movementSync.hasAttachedFirework() || rocketCooldown > 35);
        }
        ElytraFlightController.Decision applied = decision;
        // Cancellation may happen while trajectory evaluation is running. Guard the actual writes too.
        runIfCurrent(() -> {
            if (isFinished()) return;
            movementSync.setYawToward(applied.yaw());
            movementSync.pitch.set(applied.pitch());
            if (applied.firework() && !movementSync.hasAttachedFirework()
                    && equipmentAction != xin.bbtt.inventory.InventoryManager.ElytraAction.WAITING) {
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
