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
    private final xin.bbtt.flight.FlightConfiguration flightConfiguration;
    private final xin.bbtt.physics.FlightTarget target;
    private long generation;
    private int flightTicks;
    private final VanillaGlideLifecycle lifecycle = new VanillaGlideLifecycle();
    private boolean emergencyLanding;
    private boolean safeLandingLeg;
    private boolean emergencyHoldWarned;
    private boolean independentLanding;
    private final xin.bbtt.physics.LandingSearch landingSearch = new xin.bbtt.physics.LandingSearch();

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
        this(new xin.bbtt.physics.FixedFlightTarget(target), generation, arrivalRadius);
    }

    public ElytraFlyMovement(Vector3d target, xin.bbtt.flight.FlightContextBuilder builder) {
        this(target, MovementSync.INSTANCE.getNavigationGeneration(), DEFAULT_ARRIVAL_RADIUS, builder);
    }

    public ElytraFlyMovement(Vector3d target, long generation, xin.bbtt.flight.FlightContextBuilder builder) {
        this(target, generation, DEFAULT_ARRIVAL_RADIUS, builder);
    }

    public ElytraFlyMovement(Vector3d target, long generation, double arrivalRadius,
                             xin.bbtt.flight.FlightContextBuilder builder) {
        this(new xin.bbtt.physics.FixedFlightTarget(target), generation, arrivalRadius, builder);
    }

    public ElytraFlyMovement(xin.bbtt.physics.FlightTarget target, long generation, double arrivalRadius) {
        this(target, generation, arrivalRadius, new xin.bbtt.flight.FlightContextBuilder().addStandard());
    }

    public ElytraFlyMovement(xin.bbtt.physics.FlightTarget target, long generation, double arrivalRadius,
                             xin.bbtt.flight.FlightContextBuilder builder) {
        if (!Double.isFinite(arrivalRadius) || arrivalRadius <= 0) throw new IllegalArgumentException("Radius must be positive");
        this.arrivalRadius = arrivalRadius;
        this.target = java.util.Objects.requireNonNull(target);
        this.generation = generation;
        this.flightConfiguration = java.util.Objects.requireNonNull(builder).build();
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
        return target.position();
    }

    @Override
    public void init() {
        MovementSync movementSync = MovementSync.INSTANCE;
        if (!runIfCurrent(() -> {
            if (isFinished()) return;
            lifecycle.init(movementSync);
        })) setFinished(true);
    }

    /** Entity fluid-height contact using the current player body, not a block-depth threshold. */
    static boolean touchingWater() {
        var sync = MovementSync.INSTANCE;
        return xin.bbtt.physics.EnvironmentalPhysics.sample(sync.getWorld(), sync.position.get(),
                (double) 0.6f / 2, sync.isGliding() ? (double) 0.6f : (double) 1.8f).waterHeight() > 0;
    }

    @Override
    public void onTick() {
        if (isFinished()) return;
        MovementSync movementSync = MovementSync.INSTANCE;
        if (!runIfCurrent(() -> {})) {
            setFinished(true);
            return;
        }
        if (lifecycle.awaitingTakeoff(movementSync)) {
            runIfCurrent(() -> {
                if (isFinished()) return;
                lifecycle.tickTakeoff(movementSync, () -> finish(false));
            });
            return;
        }
        lifecycle.markAirborne();
        PhysicsState state = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(), movementSync.onGround.get(), movementSync.fallDistance.get());
        // One observation per second limits collision-column work during flight.
        if (state.onGround() || flightTicks % 20 == 0)
            runIfCurrent(() -> movementSync.getLandingMemory().observe(movementSync.getWorld(), state.position()));
        boolean landed = state.onGround() || touchingWater();
        if (landed || !movementSync.isGliding() || ++flightTicks > 12000) {
            runIfCurrent(() -> {
                if (isFinished()) return;
                Vector3d position = state.position();
                if (state.onGround()) movementSync.getLandingMemory().landed(movementSync.getWorld(), position);
                if (safeLandingLeg && landed) {
                    setFinished(true);
                    MovementSync.getLogger().info("Elytra safe landing complete at {}", position);
                } else {
                    finish(!emergencyLanding && landed
                            && Math.hypot(position.x - target.position().x, position.z - target.position().z) <= arrivalRadius);
                }
            });
            return;
        }
        lifecycle.tickCooldown();
        var equipmentAction = movementSync.inventoryManager.tickElytraSwap(false,
                movementSync::queueBeforeMovement, () -> xin.bbtt.mcbot.Bot.INSTANCE.getAndIncreaseSequence(),
                movementSync.yaw.get(), movementSync.pitch.get());
        // Extended configurations arbitrate all owners before vanilla-only emergency checks.
        // In particular, exhausted rockets or low elytra durability need not disable a plugin mode.
        if (!flightConfiguration.isLegacy() && tickExtended(movementSync, state, equipmentAction)) return;
        ElytraFlightController.Decision decision;
        try {
            xin.bbtt.physics.MovementParameters parameters = movementSync.getMovementParameters();
            ElytraFlightController controller = new ElytraFlightController(parameters,
                    xin.bbtt.physics.DamageParameters.fromPlayer(movementSync.inventoryManager,
                            movementSync.getMovementParameterState().effectsSnapshot()),
                    movementSync.getMovementParameterState().health());
            if (!emergencyLanding && equipmentAction == xin.bbtt.inventory.InventoryManager.ElytraAction.LAND) {
                // Spare-equipment replacement must be attempted before latching this emergency.
                emergencyLanding = true;
                MovementSync.getLogger().warn("Elytra durability low; seeking solid emergency landing ground");
            }
            boolean attached = movementSync.hasAttachedFirework() || lifecycle.rocketCooldown() > 35;
            int fireworks = lifecycle.rocketCooldown() == 0 ? movementSync.countHotbarFireworks() : 0;
            int[] rocketFlights = lifecycle.rocketCooldown() == 0
                    ? movementSync.inventoryManager.getHotbarFireworkFlightDurations() : new int[0];
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
                        movementSync.yaw.get(), movementSync.pitch.get(), attached, rocketFlights, arrivalRadius,
                        movementSync.getLandingMemory(), landingSearch);
            } else {
                try {
                    selected = controller.decide(movementSync.getWorld(), state,
                            movementSync.yaw.get(), movementSync.pitch.get(), attached, target, fireworks, arrivalRadius);
                } catch (IllegalStateException unavailable) {
                    emergencyLanding = true;
                    selected = controller.forceLand(movementSync.getWorld(), state,
                            movementSync.yaw.get(), movementSync.pitch.get(), attached, rocketFlights, arrivalRadius,
                            movementSync.getLandingMemory(), landingSearch);
                }
                if (!emergencyLanding && selected.phase() == ElytraFlightController.Phase.FORCED_LANDING) {
                    emergencyLanding = true;
                    selected = controller.forceLand(movementSync.getWorld(), state,
                            movementSync.yaw.get(), movementSync.pitch.get(), attached, rocketFlights, arrivalRadius,
                            movementSync.getLandingMemory(), landingSearch);
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
            decision = new ElytraFlightController(parameters,
                    xin.bbtt.physics.DamageParameters.fromPlayer(movementSync.inventoryManager,
                            movementSync.getMovementParameterState().effectsSnapshot()),
                    movementSync.getMovementParameterState().health())
                    .emergencyHold(movementSync.getWorld(), state, movementSync.yaw.get(), movementSync.pitch.get(),
                            movementSync.hasAttachedFirework() || lifecycle.rocketCooldown() > 35);
        }
        if (emergencyLanding && flightTicks % 5 == 0) {
            MovementSync.getLogger().debug("Emergency steering at {} velocity {} fallDistance={} yaw={} pitch={}",
                    state.position(), state.velocity(), state.fallDistance(), decision.yaw(), decision.pitch());
        }
        ElytraFlightController.Decision applied = decision;
        // Cancellation may happen while trajectory evaluation is running. Guard the actual writes too.
        runIfCurrent(() -> {
            if (isFinished()) return;
            lifecycle.execute(movementSync, applied, equipmentAction);
        });
    }

    /** Extensions never fall through to unchecked vanilla controls when their safety rules reject all candidates. */
    private boolean tickExtended(MovementSync sync, PhysicsState state,
                                 xin.bbtt.inventory.InventoryManager.ElytraAction equipmentAction) {
        var inventory = new java.util.HashMap<String, Integer>();
        var stacks = sync.inventoryManager.getInventory();
        if (stacks != null) for (var stack : stacks) {
            if (stack == null || stack.getAmount() <= 0) continue;
            var item = xin.bbtt.inventory.ItemRegistry.Instance.getItem(stack.getId());
            if (item != null) inventory.merge("minecraft:" + item.getName(), stack.getAmount(), Integer::sum);
        }
        var context = extendedContext(sync, state, inventory, emergencyLanding);
        xin.bbtt.flight.FlightSelector.Selection selected;
        try {
            selected = flightConfiguration.select(context);
        } catch (IllegalStateException unavailable) {
            if (emergencyLanding) {
                runIfCurrent(() -> finish(false));
                return true;
            }
            // Retry every owner with damage-first emergency ranking before invoking vanilla fallback.
            context = extendedContext(sync, state, inventory, true);
            try {
                selected = flightConfiguration.select(context);
            } catch (IllegalStateException noEmergencyCandidate) {
                runIfCurrent(() -> finish(false));
                return true;
            }
        }
        var appliedContext = context;
        var applied = selected;
        runIfCurrent(() -> {
            if (isFinished()) return;
            if (appliedContext.emergency()) emergencyLanding = true;
            if (applied.mode() instanceof xin.bbtt.flight.VanillaGlideMode) {
                var glide = (xin.bbtt.flight.FlightAction.Glide) applied.action();
                // Standard execution must retain the existing equipment wait and rocket cooldown latch.
                lifecycle.execute(sync, new ElytraFlightController.Decision(glide.yaw(), glide.pitch(),
                        glide.firework(), emergencyLanding ? ElytraFlightController.Phase.FORCED_LANDING
                                : ElytraFlightController.Phase.CRUISE), equipmentAction);
            } else {
                applied.mode().execute(appliedContext, applied.action(), sync);
            }
        });
        return true;
    }

    private xin.bbtt.flight.FlightContext extendedContext(MovementSync sync, PhysicsState state,
                                                         java.util.Map<String, Integer> inventory,
                                                         boolean emergency) {
        return new xin.bbtt.flight.FlightContext(sync.getWorld(), state, sync.getMovementParameters(),
                xin.bbtt.physics.DamageParameters.fromPlayer(sync.inventoryManager,
                        sync.getMovementParameterState().effectsSnapshot()),
                sync.getMovementParameterState().health(), target.aim(state), sync.yaw.get(), sync.pitch.get(),
                lifecycle.rocketCooldown() == 0 ? sync.countHotbarFireworks() : 0,
                sync.hasAttachedFirework() || lifecycle.rocketCooldown() > 35, arrivalRadius,
                emergency, null, inventory);
    }

    private void finish(boolean arrived) {
        setFinished(true);
        MovementSync.getLogger().info(LangManager.get(arrived
                ? "movementsync.command.elytra.fly.arrived" : "movementsync.command.elytra.fly.failed"));
        if (!arrived) {
            Vector3d position = MovementSync.INSTANCE.position.get();
            MovementSync.getLogger().info("Flight failed: remaining horizontal distance={} blocks, arrival radius={} blocks",
                    Math.hypot(position.x - target.position().x, position.z - target.position().z), arrivalRadius);
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
