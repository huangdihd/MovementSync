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
    private xin.bbtt.physics.FlightTarget target;
    private boolean targetBelowRoof;
    private xin.bbtt.physics.FlightTarget roofGoal;
    private boolean completedAboveRoof;

    /** Whether this leg landed at the roof substitute for an inaccessible static goal. */
    boolean completedAboveRoof() { return completedAboveRoof; }
    private long generation;
    private int flightTicks;
    private xin.bbtt.flight.FlightMode controllingMode;
    private xin.bbtt.flight.FlightTransitions.Selection transition;
    private xin.bbtt.flight.FlightTransitions.Kind transitionKind;
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
        if (isFinished()) return;
        if (!flightConfiguration.isLegacy() && !runIfCurrent(() -> {})) {
            setFinished(true);
            return;
        }
        if (!flightConfiguration.isLegacy() && !movementSync.isGliding()) {
            var state = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(),
                    movementSync.onGround.get(), movementSync.fallDistance.get());
            if (startTransition(movementSync, state, xin.bbtt.flight.FlightTransitions.Kind.TAKEOFF)) return;
        }
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
            onStop();
            return;
        }
        projectRoofTarget(movementSync);
        if (transition != null) {
            runIfCurrent(() -> {
                if (isFinished() || transition == null) return;
                var active = transition;
                if (!active.movement().isFinished()) active.movement().onTick();
                if (active.movement().isFinished()) {
                    transition = null;
                    active.movement().onStop();
                    controllingMode = active.mode();
                    if (transitionKind == xin.bbtt.flight.FlightTransitions.Kind.TAKEOFF) lifecycle.markAirborne();
                }
            });
            return;
        }
        PhysicsState state = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(), movementSync.onGround.get(), movementSync.fallDistance.get());
        // The legacy policy keeps its per-tick work: no context snapshot, gliding ends the flight.
        var airborneContext = flightConfiguration.isLegacy() ? null
                : extendedContext(movementSync, state, inventorySnapshot(movementSync), emergencyLanding);
        if (airborneContext != null && controllingMode == null && !state.onGround()) {
            for (var mode : flightConfiguration.modes()) {
                if (mode.isAvailable(airborneContext) && mode.isAirborne(airborneContext, movementSync)) {
                    controllingMode = mode;
                    break;
                }
            }
        }
        if (controllingMode == null && lifecycle.awaitingTakeoff(movementSync)) {
            runIfCurrent(() -> {
                if (isFinished()) return;
                lifecycle.tickTakeoff(movementSync, () -> finish(false));
            });
            return;
        }
        lifecycle.markAirborne();
        // One observation per second limits collision-column work during flight.
        if (state.onGround() || flightTicks % 20 == 0)
            runIfCurrent(() -> movementSync.getLandingMemory().observe(movementSync.getWorld(), state.position()));
        boolean landed = state.onGround() || touchingWater();
        boolean airborne = controllingMode == null || airborneContext == null ? movementSync.isGliding()
                : controllingMode.isAirborne(airborneContext, movementSync);
        // No time limit: a long route ends on arrival, landing, or the durability and rocket budget.
        flightTicks++;
        if (landed || !airborne) {
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
        if (!flightConfiguration.isLegacy() && (safeLandingLeg
                || !emergencyLanding && Math.hypot(state.position().x - target.position().x,
                        state.position().z - target.position().z) <= arrivalRadius)
                && startTransition(movementSync, state, xin.bbtt.flight.FlightTransitions.Kind.LANDING)) return;
        lifecycle.tickCooldown();
        var equipmentAction = movementSync.inventoryManager.tickElytraSwap(false,
                movementSync::queueBeforeMovement, () -> xin.bbtt.mcbot.Bot.INSTANCE.getAndIncreaseSequence(),
                movementSync.yaw.get(), movementSync.pitch.get());
        // Plugin-only configurations must never acquire implicit vanilla steering.
        boolean hasVanilla = flightConfiguration.modes().stream()
                .anyMatch(mode -> mode instanceof xin.bbtt.flight.VanillaGlideMode);
        if (!hasVanilla && tickExtended(movementSync, state, equipmentAction, null)) return;
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
        // Compute vanilla landing/memory/firework-budget decisions before plugin arbitration.
        if (!flightConfiguration.isLegacy() && tickExtended(movementSync, state, equipmentAction, decision)) return;
        ElytraFlightController.Decision applied = decision;
        // Cancellation may happen while trajectory evaluation is running. Guard the actual writes too.
        runIfCurrent(() -> {
            if (isFinished()) return;
            controllingMode = xin.bbtt.flight.VanillaGlideMode.INSTANCE;
            lifecycle.execute(movementSync, applied, equipmentAction);
        });
    }

    /** Vanilla competes in ranking but executes through the retained lifecycle, never a recursive child. */
    private boolean startTransition(MovementSync sync, PhysicsState state,
                                    xin.bbtt.flight.FlightTransitions.Kind kind) {
        var context = extendedContext(sync, state, inventorySnapshot(sync), emergencyLanding);
        boolean hasPlugin = flightConfiguration.modes().stream().anyMatch(mode ->
                !(mode instanceof xin.bbtt.flight.VanillaGlideMode) && mode.isAvailable(context));
        if (!hasPlugin) return false;
        var selected = xin.bbtt.flight.FlightTransitions.select(flightConfiguration, context, kind);
        if (selected == null || selected.mode() instanceof xin.bbtt.flight.VanillaGlideMode) return false;
        if (!runIfCurrent(() -> {
            if (isFinished()) return;
            transition = selected;
            transitionKind = kind;
            if (selected.movement() instanceof NavigationBoundMovement bound) bound.bindNavigationRequest(generation);
            selected.movement().init();
        })) setFinished(true);
        return true;
    }

    /** Extensions never fall through to unchecked vanilla controls when their safety rules reject all candidates. */
    private boolean tickExtended(MovementSync sync, PhysicsState state,
                                 xin.bbtt.inventory.InventoryManager.ElytraAction equipmentAction,
                                 ElytraFlightController.Decision vanillaDecision) {
        var inventory = inventorySnapshot(sync);
        var context = extendedContext(sync, state, inventory, emergencyLanding);
        // An unavailable registration must not change even emergencyHold's exact controls.
        var vanillaPolicy = new xin.bbtt.flight.FlightConfiguration(flightConfiguration.modes().stream()
                .filter(mode -> mode instanceof xin.bbtt.flight.VanillaGlideMode).toList(),
                flightConfiguration.costs(), flightConfiguration.damageRules(), flightConfiguration.cruiseDamageFirst());
        if (vanillaDecision != null && vanillaPolicy.isLegacy()) {
            boolean pluginAvailable = false;
            for (var mode : flightConfiguration.modes()) {
                if (!(mode instanceof xin.bbtt.flight.VanillaGlideMode) && mode.isAvailable(context)) {
                    pluginAvailable = true;
                    break;
                }
            }
            if (!pluginAvailable) return false;
        }
        var configuration = flightConfiguration;
        xin.bbtt.flight.FlightMode landingOwner = null;
        if (vanillaDecision != null && emergencyLanding) {
            var decision = vanillaDecision;
            landingOwner = new xin.bbtt.flight.FlightMode() {
                @Override public java.util.List<xin.bbtt.flight.FlightAction.Glide> candidates(xin.bbtt.flight.FlightContext c) {
                    return java.util.List.of(new xin.bbtt.flight.FlightAction.Glide(decision.yaw(), decision.pitch(), decision.firework()));
                }
                @Override public xin.bbtt.flight.Trajectory predict(xin.bbtt.flight.FlightContext c, xin.bbtt.flight.FlightAction a) {
                    return xin.bbtt.flight.VanillaGlideMode.INSTANCE.predict(c, a);
                }
                @Override public java.util.List<xin.bbtt.flight.Trajectory> recover(xin.bbtt.flight.FlightContext c,
                        xin.bbtt.flight.FlightAction a, xin.bbtt.flight.Trajectory main) {
                    return xin.bbtt.flight.VanillaGlideMode.INSTANCE.recover(c, a, main);
                }
                @Override public void execute(xin.bbtt.flight.FlightContext c, xin.bbtt.flight.FlightAction a, MovementSync client) {
                    lifecycle.execute(client, decision, equipmentAction);
                }
            };
            var owner = landingOwner;
            configuration = new xin.bbtt.flight.FlightConfiguration(flightConfiguration.modes().stream()
                    .map(mode -> mode instanceof xin.bbtt.flight.VanillaGlideMode ? owner : mode).toList(),
                    flightConfiguration.costs(), flightConfiguration.damageRules(), flightConfiguration.cruiseDamageFirst());
        }
        xin.bbtt.flight.FlightSelector.Selection selected;
        try {
            selected = configuration.select(context);
        } catch (IllegalStateException unavailable) {
            if (emergencyLanding) {
                // The legacy controller deliberately retains emergencyHold when terrain is unknown.
                // Do not let an available but nonviable plugin remove that last-resort steering.
                // Custom policy must still reject candidates rather than bypass its safety rules.
                if (vanillaDecision != null && vanillaPolicy.isLegacy()) return false;
                runIfCurrent(() -> finish(false));
                return true;
            }
            if (vanillaDecision != null) {
                emergencyLanding = true;
                var controller = new ElytraFlightController(sync.getMovementParameters(),
                        context.damage(), context.health());
                ElytraFlightController.Decision landing;
                boolean attached = sync.hasAttachedFirework() || lifecycle.rocketCooldown() > 35;
                try {
                    landing = controller.forceLand(sync.getWorld(), state, sync.yaw.get(), sync.pitch.get(),
                            attached, lifecycle.rocketCooldown() == 0
                                    ? sync.inventoryManager.getHotbarFireworkFlightDurations() : new int[0],
                            arrivalRadius, sync.getLandingMemory(), landingSearch);
                } catch (IllegalStateException noLanding) {
                    landing = controller.emergencyHold(sync.getWorld(), state, sync.yaw.get(), sync.pitch.get(), attached);
                }
                // Rebuild arbitration around forceLand, never generic emergency vanilla candidates.
                if (!tickExtended(sync, state, equipmentAction, landing)) {
                    var fallback = landing;
                    runIfCurrent(() -> {
                        if (isFinished()) return;
                        controllingMode = xin.bbtt.flight.VanillaGlideMode.INSTANCE;
                        lifecycle.execute(sync, fallback, equipmentAction);
                    });
                }
                return true;
            }
            // Plugin-only configurations retry their owners without acquiring vanilla controls.
            context = extendedContext(sync, state, inventory, true);
            try {
                selected = configuration.select(context);
            } catch (IllegalStateException noEmergencyCandidate) {
                runIfCurrent(() -> finish(false));
                return true;
            }
        }
        var appliedContext = context;
        var applied = selected;
        var vanillaLandingOwner = landingOwner;
        runIfCurrent(() -> {
            if (isFinished()) return;
            if (appliedContext.emergency()) emergencyLanding = true;
            controllingMode = applied.mode() == vanillaLandingOwner
                    ? xin.bbtt.flight.VanillaGlideMode.INSTANCE : applied.mode();
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

    private static java.util.Map<String, Integer> inventorySnapshot(MovementSync sync) {
        var inventory = new java.util.HashMap<String, Integer>();
        var stacks = sync.inventoryManager.getInventory();
        if (stacks != null) for (var stack : stacks) {
            if (stack == null || stack.getAmount() <= 0) continue;
            var item = xin.bbtt.inventory.ItemRegistry.Instance.getItem(stack.getId());
            if (item != null) inventory.merge("minecraft:" + item.getName(), stack.getAmount(), Integer::sum);
        }
        return inventory;
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
                emergency, inventory);
    }

    void projectRoofTarget(MovementSync sync) {
        if (targetBelowRoof || safeLandingLeg || !(target instanceof xin.bbtt.physics.FixedFlightTarget)
                || !xin.bbtt.listeners.RegistryDataListener.hasCeiling()) return;
        var world = sync.getWorld();
        var position = sync.position.get();
        if (roofGoal == null) roofGoal = target;
        var goal = roofGoal.position();
        // Collision geometry, not logical_height, defines the roof surface. Unknown
        // columns are deferred until loaded; never invent a landing at the target Y.
        if (!xin.bbtt.physics.CeilingCollision.isAboveRoof(world, position)) return;
        var top = xin.bbtt.physics.CeilingCollision.roofTop(world, goal.x, goal.z);
        if (top.isEmpty()) {
            // Until the target column loads, approach no lower than the roof under us:
            // descending toward a goal Y that may lie below the roof ends on the roof early.
            double roof = xin.bbtt.physics.CeilingCollision.roofTop(world, position.x, position.z).orElse(goal.y);
            if (roof > goal.y && target.position().y < roof)
                target = new xin.bbtt.physics.FixedFlightTarget(new Vector3d(goal.x, roof, goal.z));
            return;
        }
        if (top.getAsDouble() <= goal.y) {
            target = roofGoal;
            return;
        }
        Vector3d best = null;
        double distance = Double.POSITIVE_INFINITY;
        // Use the existing arrival radius as the search extent, with block-centre
        // samples; landingSite retains the standard support/clearance/hazard rules.
        int radius = (int) Math.ceil(arrivalRadius);
        for (int dx = -radius; dx <= radius; dx++) {
            for (int dz = -radius; dz <= radius; dz++) {
                double squared = (double) dx * dx + (double) dz * dz;
                if (squared > arrivalRadius * arrivalRadius || squared >= distance) continue;
                var site = ElytraFlightController.landingSite(world, goal.x + dx, goal.z + dz,
                        xin.bbtt.world.World.getMaxWorldY() + 1.0);
                if (site != null && site.y >= top.getAsDouble()) {
                    best = site;
                    distance = squared;
                }
            }
        }
        if (best != null) {
            target = new xin.bbtt.physics.FixedFlightTarget(best);
            targetBelowRoof = true;
        }
    }

    private void finish(boolean arrived) {
        completedAboveRoof = arrived && targetBelowRoof;
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
        if (transition != null) {
            var active = transition;
            transition = null;
            active.movement().onStop();
        }
    }
}
