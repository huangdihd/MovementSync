package xin.bbtt.movements;

import xin.bbtt.physics.PlayerPose;
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
    private xin.bbtt.physics.WaypointFlightTarget lowRoute;
    private java.util.concurrent.CompletableFuture<LowAltitudeFlightPlan.Result> lowPlan;
    private int lowPlanTicks;
    private boolean lowAltitude;
    private boolean lowEmergencyRoute;
    private int lowRouteAcceptedTick;
    private ElytraFlightController.Decision lastLowDecision;
    private xin.bbtt.physics.PhysicsResult lastLowPrediction;
    private MovementSync.FlightTickTrace lastLowPredictionBase;
    private int lastLowAirborneProofDecisionTick = -1;
    private long lastLowPredictionPhysicsTick = -1;
    private long lastLowAirborneProofPhysicsTick = -1;

    ElytraFlyMovement withLowAltitudePlan(LowAltitudeFlightPlan.Result result) {
        lowRoute = result.route();
        if (lowRoute.pursuit()) lowRoute.follow(target, result.destination());
        lowAltitude = true;
        lifecycle.allowLowAltitudeTakeoff(result.yaw(), result.pitch());
        return this;
    }

    /** Keep corridor landing in the local planner even for a fixed target in an ordinary dimension. */
    ElytraFlyMovement withLowAltitudeLanding() {
        lowAltitude = true;
        return this;
    }

    private xin.bbtt.physics.FlightTarget steeringTarget() {
        return lowRoute == null ? target : lowRoute;
    }

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
    // Preserve an observed glide across metadata loss without treating plugin flight as vanilla takeoff.
    private boolean observedVanillaAirborne;
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
        // Independent cancellation already owns an observed glide. Scheduler init is
        // synchronous under the navigation lock: no terrain search or takeoff here.
        if (independentLanding) {
            lifecycle.markAirborne();
            return;
        }
        if (!flightConfiguration.isLegacy() && !runIfCurrent(() -> {})) {
            setFinished(true);
            return;
        }
        if (!lowAltitude && !flightConfiguration.isLegacy() && !movementSync.isGliding()) {
            var state = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(),
                    movementSync.onGround.get(), movementSync.fallDistance.get());
            if (startTransition(movementSync, state, xin.bbtt.flight.FlightTransitions.Kind.TAKEOFF)) return;
        }
        if (!runIfCurrent(() -> {
            if (isFinished()) return;
            lifecycle.init(movementSync);
            observedVanillaAirborne = movementSync.isGliding();
        })) setFinished(true);
    }

    /** Entity fluid-height contact using the current player body, not a block-depth threshold. */
    static boolean touchingWater() {
        var sync = MovementSync.INSTANCE;
        return xin.bbtt.physics.EnvironmentalPhysics.sample(sync.getWorld(), sync.position.get(),
                PlayerPose.HALF_WIDTH, sync.isGliding() ? PlayerPose.GLIDING.height() : (double) 1.8f).waterHeight() > 0;
    }

    @Override
    public void onTick() {
        if (isFinished()) return;
        MovementSync movementSync = MovementSync.INSTANCE;
        if (!runIfCurrent(() -> {})) {
            onStop();
            return;
        }
        // Glide metadata can clear before physical touchdown (including between
        // safety publication and init). Keep this same owner; do not restart glide
        // or mistake a flight-mode transition for a supported stop.
        if (safeLandingLeg && !movementSync.isGliding()
                && (independentLanding || controllingMode == null && observedVanillaAirborne)) {
            tickNonglidingLanding(movementSync);
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
        var controlSource = xin.bbtt.physics.FlightControlAdmission.snapshot(movementSync);
        PhysicsState state = controlSource.state();
        diagnoseLowFlight(movementSync, state);
        updateLowRoute(movementSync, state);
        // The legacy policy keeps its per-tick work: no context snapshot, gliding ends the flight.
        var airborneContext = lowAltitude || flightConfiguration.isLegacy() ? null
                : extendedContext(movementSync, state, inventorySnapshot(movementSync), emergencyLanding);
        if (airborneContext != null && controllingMode == null && !state.onGround()) {
            for (var mode : flightConfiguration.modes()) {
                if (mode.isAvailable(airborneContext) && mode.isAirborne(airborneContext, movementSync)) {
                    controllingMode = mode;
                    break;
                }
            }
        }
        if (controllingMode == null && !observedVanillaAirborne && lifecycle.awaitingTakeoff(movementSync)) {
            runIfCurrent(() -> {
                if (isFinished()) return;
                lifecycle.tickTakeoff(movementSync, () -> finish(false));
            });
            return;
        }
        observedVanillaAirborne |= movementSync.isGliding();
        lifecycle.markAirborne();
        // One observation per second limits collision-column work during flight.
        if (state.onGround() || flightTicks % 20 == 0)
            runIfCurrent(() -> movementSync.getLandingMemory().observe(movementSync.getWorld(), state.position()));
        boolean landed = state.onGround() || touchingWater();
        boolean airborne = controllingMode == null || airborneContext == null ? movementSync.isGliding()
                : controllingMode.isAirborne(airborneContext, movementSync);
        // No time limit: a long route ends on arrival, landing, or the durability and rocket budget.
        flightTicks++;
        if (state.onGround() && hazardousGroundHandoff(movementSync)) {
            if (!movementSync.isGliding()) {
                runIfCurrent(() -> GroundedFlightStop.hold(movementSync));
                if (!GroundedFlightStop.stopped(movementSync)) return;
            } else {
                // Selector treats onGround as LANDED (yaw=0); bypass it until metadata clears.
                runIfCurrent(() -> {
                    if (!GroundedFlightStop.steerGroundedGlide(movementSync) && flightTicks % 20 == 0)
                        MovementSync.getLogger().warn("Grounded glide stop unproved: position={}, velocity={}; retaining physical braking, awaiting metadata",
                                movementSync.position.get(), movementSync.velocity.get());
                });
                return;
            }
        }
        final boolean landedNow = landed;
        if (landedNow || !airborne) {
            runIfCurrent(() -> {
                if (isFinished()) return;
                Vector3d position = state.position();
                if (state.onGround()) movementSync.getLandingMemory().landed(movementSync.getWorld(), position);
                if (safeLandingLeg && landedNow) {
                    if (independentLanding && hazardousGroundHandoff(movementSync)
                            && !GroundedFlightStop.stopped(movementSync)) return;
                    setFinished(true);
                    MovementSync.getLogger().info("Elytra safe landing complete at {}", position);
                } else {
                    MovementSync.getLogger().info("Flight lifecycle ended: reason={}, follow={}, onGround={}, water={}, airborne={}, position={}, velocity={}, lowAltitude={}, lowRoute={}, rocketCooldown={}, flightTicks={}",
                            landedNow ? "ground-or-water-contact" : "gliding-state-ended",
                            target instanceof xin.bbtt.physics.EntityFlightTarget,
                            state.onGround(), touchingWater(), airborne, position, state.velocity(),
                            lowAltitude, lowRoute != null, lifecycle.rocketCooldown(), flightTicks);
                    finish(!emergencyLanding && !(target instanceof xin.bbtt.physics.EntityFlightTarget entity && !entity.gliding())
                            && landedNow && Math.abs(position.y - target.position().y) <= 1.0
                            && Math.hypot(position.x - target.position().x, position.z - target.position().z) <= arrivalRadius);
                }
            });
            return;
        }
        if (!lowAltitude && !flightConfiguration.isLegacy() && (safeLandingLeg
                || !emergencyLanding && Math.hypot(state.position().x - target.position().x,
                        state.position().z - target.position().z) <= arrivalRadius)
                && startTransition(movementSync, state, xin.bbtt.flight.FlightTransitions.Kind.LANDING)) return;
        lifecycle.tickCooldown(movementSync);
        var equipmentAction = movementSync.inventoryManager.tickElytraSwap(false,
                movementSync::queueBeforeMovement, () -> xin.bbtt.mcbot.Bot.INSTANCE.getAndIncreaseSequence(),
                movementSync.yaw.get(), movementSync.pitch.get());
        // Plugin-only configurations must never acquire implicit vanilla steering.
        boolean hasVanilla = flightConfiguration.modes().stream()
                .anyMatch(mode -> mode instanceof xin.bbtt.flight.VanillaGlideMode);
        if (!hasVanilla && tickExtended(movementSync, state, equipmentAction, null)) return;
        // A stalled static corridor leg above harm cannot wait for asynchronous replanning.
        // This runs before emergencyHold, whose unpowered descent is not a rescue from lava.
        if (hasVanilla && lowAltitude && !(target instanceof xin.bbtt.physics.EntityFlightTarget)
                && !state.onGround()
                && harmBelow(movementSync.getWorld(), state)
                && (emergencyLanding || lowRoute == null || Math.hypot(state.velocity().x, state.velocity().z) < .3)) {
            var context = extendedContext(movementSync, state, inventorySnapshot(movementSync), true);
            var rescue = lowHazardDecision(context, flightConfiguration, recoveryContext(movementSync));
            if (flightTicks % 20 == 1 || rescue == null) {
                MovementSync.getLogger().warn("Low flight hazard rescue: position={}, velocity={}, onGround={}, route={}, attached={}, availableNow={}, inventoryRockets={}, cooldownTicks={}, selected={}, lastPhysicsTick={}",
                        state.position(), state.velocity(), state.onGround(), lowRoute != null,
                        context.attached(), context.fireworks(), movementSync.countHotbarFireworks(),
                        lifecycle.rocketCooldown(), rescue, movementSync.getLastFlightTick());
            }
            if (rescue != null) {
                var rescueEnvelope = xin.bbtt.physics.FlightControlAdmission.prepare(movementSync,
                        this, controlSource, rescue, null,
                        () -> lifecycle.execute(movementSync, rescue, equipmentAction));
                runIfCurrent(() -> {
                    if (isFinished()) return;
                    emergencyLanding = true;
                    controllingMode = xin.bbtt.flight.VanillaGlideMode.INSTANCE;
                    movementSync.submitFlightControls(rescueEnvelope);
                    predictLowDecision(movementSync, state, rescue);
                });
                return;
            }
            // Never relinquish airborne ownership when rescue is unproven.
            // Retain damage-first steering and retry as terrain/state changes.
        }
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
            boolean attached = movementSync.hasAttachedFirework();
            int fireworks = lifecycle.rocketCooldown() == 0 ? movementSync.countHotbarFireworks() : 0;
            int[] rocketFlights = lifecycle.rocketCooldown() == 0
                    ? movementSync.inventoryManager.getHotbarFireworkFlightDurations() : new int[0];
            if (!emergencyLanding && !attached && movementSync.countHotbarFireworks() == 0
                    && "minecraft:the_end".equals(movementSync.getRecoveryDimension())) {
                Vector3d p = state.position();
                var support = movementSync.getWorld().findHighestCollisionTop(p.x - PlayerPose.HALF_WIDTH,
                        xin.bbtt.world.World.getMinWorldY(), p.z - PlayerPose.HALF_WIDTH, p.x + PlayerPose.HALF_WIDTH, p.y, p.z + PlayerPose.HALF_WIDTH);
                if (support.isEmpty()) {
                    emergencyLanding = true;
                    MovementSync.getLogger().warn("Fireworks exhausted above void; diverting to known solid ground");
                }
            }
            ElytraFlightController.Decision selected;
            if (lowAltitude && (lowRoute == null || emergencyLanding && !lowEmergencyRoute)) {
                // Do not wait for a worker above harm when a terminal powered exit is proven.
                var context = extendedContext(movementSync, state, inventorySnapshot(movementSync), true);
                selected = routeRejoinDecision(context, movementSync);
                if (selected == null) selected = lowPursuit() ? localPursuitDecision(context,
                        xin.bbtt.physics.WaypointFlightTarget.pursuitAim(target, state), flightConfiguration) : null;
                if (selected == null && harmBelow(movementSync.getWorld(), state)) {
                    selected = lowHazardDecision(context, flightConfiguration, recoveryContext(movementSync));
                }
                if (selected == null) selected = controller.emergencyHold(movementSync.getWorld(), state,
                        movementSync.yaw.get(), movementSync.pitch.get(), attached);
            } else if (emergencyLanding && !lowAltitude) {
                selected = controller.forceLand(movementSync.getWorld(), state,
                        movementSync.yaw.get(), movementSync.pitch.get(), attached, rocketFlights, arrivalRadius,
                        movementSync.getLandingMemory(), landingSearch);
            } else {
                try {
                    if (lowAltitude && lowRoute != null) {
                        var context = extendedContext(movementSync, state, inventorySnapshot(movementSync), emergencyLanding);
                        var recovery = recoveryContext(movementSync);
                        selected = controller.decide(movementSync.getWorld(), state,
                                movementSync.yaw.get(), movementSync.pitch.get(), attached, lowRoute, fireworks, arrivalRadius,
                                recovery,
                                candidate -> lowDecisionSafe(context, candidate, flightConfiguration, recovery)
                                        && (!lowPursuit() || airbornePursuitPreview(context, candidate, flightConfiguration) != null));
                    } else {
                        selected = controller.decide(movementSync.getWorld(), state,
                                movementSync.yaw.get(), movementSync.pitch.get(), attached, steeringTarget(), fireworks, arrivalRadius);
                    }
                } catch (IllegalStateException unavailable) {
                    if (!lowAltitude) emergencyLanding = true;
                    if (lowAltitude) {
                        var context = extendedContext(movementSync, state, inventorySnapshot(movementSync), true);
                        // A stale/exhausted corridor is not a request to land. On ordinary
                        // mountain support HazardRecovery is NOT_EXPOSED and has no control
                        // to return; try a newly swept local pursuit before emergency descent.
                        selected = routeRejoinDecision(context, movementSync);
                        if (selected == null) selected = lowPursuit() ? localPursuitDecision(context,
                                xin.bbtt.physics.WaypointFlightTarget.pursuitAim(target, state), flightConfiguration) : null;
                        if (selected != null && !lowDecisionSafe(context, selected, flightConfiguration,
                                recoveryContext(movementSync))) selected = null;
                        if (selected == null)
                            selected = lowHazardDecision(context, flightConfiguration, recoveryContext(movementSync));
                        if (selected == null) throw unavailable;
                    } else {
                        selected = controller.forceLand(movementSync.getWorld(), state,
                                movementSync.yaw.get(), movementSync.pitch.get(), attached, rocketFlights, arrivalRadius,
                                movementSync.getLandingMemory(), landingSearch);
                    }
                }
                if (!lowAltitude && !emergencyLanding && selected.phase() == ElytraFlightController.Phase.FORCED_LANDING) {
                    emergencyLanding = true;
                    selected = controller.forceLand(movementSync.getWorld(), state,
                            movementSync.yaw.get(), movementSync.pitch.get(), attached, rocketFlights, arrivalRadius,
                            movementSync.getLandingMemory(), landingSearch);
                }
            }
            if (lowAltitude && !emergencyLanding
                    && selected.phase() == ElytraFlightController.Phase.FORCED_LANDING) {
                var rejoin = routeRejoinDecision(extendedContext(movementSync, state,
                        inventorySnapshot(movementSync), false), movementSync);
                if (rejoin != null) selected = rejoin;
            }
            if (lowPursuit()) {
                var context = extendedContext(movementSync, state, inventorySnapshot(movementSync), false);
                var local = retainAirbornePursuit(context, selected,
                        xin.bbtt.physics.WaypointFlightTarget.pursuitAim(target, state), flightConfiguration);
                // Only fall back to damage-first landing when no known airborne control exists.
                if (local != null) selected = local;
            }
            decision = selected;
        } catch (IllegalStateException error) {
            if (!lowAltitude) emergencyLanding = true;
            if (!emergencyHoldWarned) {
                MovementSync.getLogger().warn("Emergency landing unavailable at {} with velocity {}: {}; "
                                + "trying proven route rejoin before last-resort steering; safe touchdown is not guaranteed",
                        state.position(), state.velocity(), error.getMessage());
                emergencyHoldWarned = true;
            }
            var parameters = movementSync.getMovementParameters();
            decision = lowAltitude ? routeRejoinDecision(
                    extendedContext(movementSync, state, inventorySnapshot(movementSync), emergencyLanding), movementSync) : null;
            if (decision == null) decision = new ElytraFlightController(parameters,
                    xin.bbtt.physics.DamageParameters.fromPlayer(movementSync.inventoryManager,
                            movementSync.getMovementParameterState().effectsSnapshot()),
                    movementSync.getMovementParameterState().health())
                    .emergencyHold(movementSync.getWorld(), state, movementSync.yaw.get(), movementSync.pitch.get(),
                            movementSync.hasAttachedFirework());
        }
        if (emergencyLanding && flightTicks % 5 == 0) {
            MovementSync.getLogger().debug("Emergency steering at {} velocity {} fallDistance={} yaw={} pitch={}",
                    state.position(), state.velocity(), state.fallDistance(), decision.yaw(), decision.pitch());
        }
        // Compute vanilla landing/memory/firework-budget decisions before plugin arbitration.
        if (!lowAltitude && !flightConfiguration.isLegacy() && tickExtended(movementSync, state, equipmentAction, decision)) return;
        if (lowAltitude && !lowDecisionSafe(
                extendedContext(movementSync, state, inventorySnapshot(movementSync), emergencyLanding), decision, flightConfiguration, recoveryContext(movementSync))) {
            var context = extendedContext(movementSync, state, inventorySnapshot(movementSync), true);
            var fallback = routeRejoinDecision(context, movementSync);
            if (fallback == null) fallback = new ElytraFlightController(context.movement(), context.damage(), context.health())
                    .emergencyHold(context.world(), state, context.yaw(), context.pitch(), context.attached());
            if (lowPursuit()) {
                var local = retainAirbornePursuit(context, fallback,
                        xin.bbtt.physics.WaypointFlightTarget.pursuitAim(target, state), flightConfiguration);
                if (local != null) fallback = local;
            }
            if (!lowDecisionSafe(context, fallback, flightConfiguration, recoveryContext(movementSync))) {
                // A terminal recovery proves its changing controls, not six ticks
                // of its first input. Revalidate that proof instead of abandoning
                // airborne ownership when the constant-input preview fails.
                var rescue = lowHazardDecision(context, flightConfiguration, recoveryContext(movementSync));
                if (rescue != null) fallback = rescue;
            }
            decision = fallback;
        }
        // The generic safety gate permits harmless solid-ground touchdown. Pursuit does not:
        // recheck after every fallback (including exceptions and the final safety replacement),
        // rather than trusting the decision's phase label or presence of a stale corridor.
        if (lowPursuit()) {
            var context = extendedContext(movementSync, state, inventorySnapshot(movementSync), false);
            var airborneControl = retainAirbornePursuit(context, decision,
                    xin.bbtt.physics.WaypointFlightTarget.pursuitAim(target, state), flightConfiguration);
            if (airborneControl != null && lowDecisionSafe(context, airborneControl, flightConfiguration, recoveryContext(movementSync))) {
                decision = airborneControl;
            }
        }
        ElytraFlightController.Decision applied = decision;
        var envelope = lowAltitude ? xin.bbtt.physics.FlightControlAdmission.prepare(movementSync,
                this, controlSource, applied, lowRoute,
                () -> lifecycle.execute(movementSync, applied, equipmentAction)) : null;
        // Cancellation may happen while trajectory evaluation is running. Guard the actual writes too.
        runIfCurrent(() -> {
            if (isFinished()) return;
            controllingMode = xin.bbtt.flight.VanillaGlideMode.INSTANCE;
            if (envelope != null) movementSync.submitFlightControls(envelope);
            else {
                // A low-flight lease must not overwrite the new owner's direct controls
                // at the next physics boundary (for example when a target starts gliding).
                movementSync.revokeFlightControls(this);
                lifecycle.execute(movementSync, applied, equipmentAction);
            }
            predictLowDecision(movementSync, state, applied);
        });
    }

    private void predictLowDecision(MovementSync sync, PhysicsState state, ElytraFlightController.Decision decision) {
        if (!lowAltitude) return;
        lastLowDecision = decision;
        lastLowPredictionBase = sync.getLastFlightTick();
        var frame = sync.getPlacementFrame();
        lastLowPredictionPhysicsTick = frame == null ? -1 : frame.tick();
        // Prove observed controls, not requested thrust: firework use may only be queued.
        state = new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get(), sync.fallDistance.get());
        var actual = new ElytraFlightController.Decision(sync.yaw.get(), sync.pitch.get(), false, decision.phase());
        if (airbornePursuitPreview(extendedContext(sync, state, inventorySnapshot(sync), emergencyLanding),
                actual, flightConfiguration) != null) {
            lastLowAirborneProofDecisionTick = flightTicks;
            lastLowAirborneProofPhysicsTick = lastLowPredictionPhysicsTick;
        }
        lastLowPrediction = new xin.bbtt.physics.GlidingPhysics(actual.yaw(), actual.pitch(),
                sync.hasAttachedFirework(), xin.bbtt.physics.PlayerInput.NONE,
                sync.getMovementParameters()).step(sync.getWorld(), state,
                        PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
    }

    private void diagnoseLowFlight(MovementSync sync, PhysicsState state) {
        if (!lowAltitude) return;
        var trace = sync.getLastFlightTick();
        boolean collision = trace != null && (trace.collidedX() || trace.collidedZ());
        boolean inCorridor = lowRoute != null && lowRoute.contains(state.position(), state.position());
        if (collision || lowRoute != null && !inCorridor && flightTicks % 20 == 0) {
            var projection = lowRoute == null ? null : forwardRouteProjection(lowRoute, state.position());
            MovementSync.getLogger().warn("Low flight corridor diagnostic: decisionTick={}, observedPhysicsTick={}, predictionPhysicsTick={}, lastAirborneProofPhysicsTick={}, position={}, velocity={}, lastPhysicsTraceIdentity={}, predictionBaseTraceIdentity={}, physicsAdvancedSinceDecision={}, actualPhysicsControls={}, waypointIndex={}, routeAgeDecisionTicks={}, inCorridor={}, routeProjection={}, routeDirection={}, lastDecision={}, lastAirborneProofDecisionTick={}, predictedNext={} (frame and trace sampled independently; no one-step deviation asserted)",
                    flightTicks, sync.getPlacementFrame() == null ? null : sync.getPlacementFrame().tick(),
                    lastLowPredictionPhysicsTick, lastLowAirborneProofPhysicsTick,
                    state.position(), state.velocity(), trace == null ? null : System.identityHashCode(trace),
                    lastLowPredictionBase == null ? null : System.identityHashCode(lastLowPredictionBase),
                    trace != lastLowPredictionBase, trace, lowRoute == null ? null : lowRoute.index(),
                    lowRoute == null ? null : flightTicks - lowRouteAcceptedTick, inCorridor,
                    projection == null ? null : projection.point(), projection == null ? null : projection.direction(),
                    lastLowDecision, lastLowAirborneProofDecisionTick, lastLowPrediction);
        }
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
            sync.revokeFlightControls(this);
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
                boolean attached = sync.hasAttachedFirework();
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
                        sync.revokeFlightControls(this);
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
            // Arbitration transfers control to direct vanilla/plugin execution, including
            // the synthetic vanilla landing owner. Retire any earlier low-flight lease.
            sync.revokeFlightControls(this);
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
                sync.getMovementParameterState().health(), emergency ? target.aim(state) : steeringTarget().aim(state), sync.yaw.get(), sync.pitch.get(),
                lifecycle.rocketCooldown() == 0 ? sync.countHotbarFireworks() : 0,
                sync.hasAttachedFirework(), arrivalRadius,
                emergency, inventory);
    }

    /** Restrict local vanilla controls with registered damage rules, without global cost arbitration. */
    static boolean lowDecisionSafe(xin.bbtt.flight.FlightContext context, ElytraFlightController.Decision decision,
                                   xin.bbtt.flight.FlightConfiguration configuration) {
        return lowDecisionSafe(context, decision, configuration, new xin.bbtt.physics.HazardRecovery.Context(
                java.util.stream.IntStream.range(0, context.fireworks()).map(i -> 1).toArray(),
                0, context.attached(), null));
    }

    private xin.bbtt.physics.HazardRecovery.Context recoveryContext(MovementSync sync) {
        boolean attached = sync.hasAttachedFirework();
        return new xin.bbtt.physics.HazardRecovery.Context(sync.inventoryManager.getHotbarFireworkFlightDurations(),
                lifecycle.rocketCooldown(), attached, sync.getLandingMemory(),
                lifecycle.confirmedBoostTicks(sync, attached));
    }

    static boolean lowDecisionSafe(xin.bbtt.flight.FlightContext context, ElytraFlightController.Decision decision,
            xin.bbtt.flight.FlightConfiguration configuration, xin.bbtt.physics.HazardRecovery.Context recovery) {
        if (decision.firework() && (recovery.attached() || recovery.cooldownTicks() > 0
                || recovery.rocketFlightDurations().length == 0)) return false;
        var action = new xin.bbtt.flight.FlightAction.Glide(decision.yaw(), decision.pitch(), decision.firework());
        var physics = new xin.bbtt.physics.GlidingPhysics(action.yaw(), action.pitch(),
                context.attached() || action.firework(), xin.bbtt.physics.PlayerInput.NONE, context.movement());
        var state = context.state();
        var results = new java.util.ArrayList<xin.bbtt.physics.PhysicsResult>();
        var proofBudget = new xin.bbtt.physics.HazardRecovery.Budget(
                xin.bbtt.physics.HazardRecovery.DECISION_STEP_BUDGET);
        // Share the work cap across successors, not a new 24,000-step cap per tick.
        // Six ticks matches a local planner edge; longer high-altitude predictions reject corridors.
        for (int tick = 0; tick < 6; tick++) {
            var result = physics.step(context.world(), state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            // Zero-damage wall contact still invalidates the executable corridor.
            if (result.collidedX() || result.collidedZ()) return false;
            results.add(result);
            state = new PhysicsState(result.position(), result.velocity(), result.onGround(), result.fallDistance(), result.pose());
            if (!xin.bbtt.physics.HazardRecovery.assess(context.world(), state, context.movement(),
                    recovery.afterTicks(tick + 1, decision.firework()), proofBudget).admissible()) return false;
            if (result.onGround() || result.collidedX() || result.collidedZ()) break;
        }
        return new xin.bbtt.flight.FlightSelector(configuration).safety(context,
                xin.bbtt.flight.VanillaGlideMode.INSTANCE, action,
                new xin.bbtt.flight.Trajectory(context.state(), results)).damageClass()
                == xin.bbtt.flight.FlightSelector.DamageClass.SAFE;
    }

    /** Immediate loaded danger beneath the body; no global landing search. */
    static boolean harmBelow(xin.bbtt.world.World world, PhysicsState state) {
        var p = state.position();
        for (int x = (int) Math.floor(p.x - PlayerPose.HALF_WIDTH);
                x <= (int) Math.floor(p.x + PlayerPose.HALF_WIDTH); x++) {
            for (int z = (int) Math.floor(p.z - PlayerPose.HALF_WIDTH);
                    z <= (int) Math.floor(p.z + PlayerPose.HALF_WIDTH); z++) {
                if (!world.chunkLoaded(x >> 4, z >> 4)) continue;
                for (int y = (int) Math.floor(p.y); y >= xin.bbtt.world.World.getMinWorldY(); y--) {
                    String name = world.getBlockStateAt(new Vector3d(x, y, z)).blockName();
                    if (name == null) continue;
                    if (name.startsWith("minecraft:")) name = name.substring(10);
                    if (name.equals("lava") || name.equals("fire") || name.equals("soul_fire")
                            || name.equals("magma_block") || name.equals("cactus")
                            || name.equals("campfire") || name.equals("soul_campfire")) return true;
                    if (world.isBoxColliding(x, y, z, x + 1, y + 1, z + 1)) break;
                }
            }
        }
        return false;
    }

    /** Emergency controls are the first step of a proven terminal supported landing. */
    static ElytraFlightController.Decision lowHazardDecision(xin.bbtt.flight.FlightContext context,
            xin.bbtt.flight.FlightConfiguration configuration) {
        return lowHazardDecision(context, configuration, new xin.bbtt.physics.HazardRecovery.Context(
                java.util.stream.IntStream.range(0, context.fireworks()).map(i -> 1).toArray(),
                0, context.attached(), null));
    }

    static ElytraFlightController.Decision lowHazardDecision(xin.bbtt.flight.FlightContext context,
            xin.bbtt.flight.FlightConfiguration configuration, xin.bbtt.physics.HazardRecovery.Context recovery) {
        var report = xin.bbtt.physics.HazardRecovery.assessToward(context.world(), context.state(),
                context.movement(), recovery, context.target(),
                candidate -> recoveryRulesSafe(context, configuration, recovery, candidate));
        // The recovery proof handles vanilla collision/hazard safety. Registered damage rules
        // still veto the executable prefix; an airborne prefix alone is never a landing proof.
        var decision = report.decision();
        return decision != null && recoveryRulesSafe(context, configuration, recovery, decision) ? decision : null;
    }

    private static boolean recoveryRulesSafe(xin.bbtt.flight.FlightContext context,
            xin.bbtt.flight.FlightConfiguration configuration, xin.bbtt.physics.HazardRecovery.Context recovery,
            ElytraFlightController.Decision decision) {
        var action = new xin.bbtt.flight.FlightAction.Glide(decision.yaw(), decision.pitch(), decision.firework());
        var next = new xin.bbtt.physics.GlidingPhysics(decision.yaw(), decision.pitch(),
                recovery.attached() || decision.firework(), xin.bbtt.physics.PlayerInput.NONE, context.movement())
                .step(context.world(), context.state(), PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
        for (var rule : configuration.damageRules()) {
            var damage = rule.damage(context, action, context.state(), next);
            if (!Double.isFinite(damage.raw()) || !Double.isFinite(damage.reduced())
                    || damage.raw() > 0 || damage.reduced() > 0) return false;
        }
        return true;
    }

    record RouteProjection(Vector3d point, Vector3d direction, Vector3d aim) {}

    /** Nearest remaining segment, never a segment behind the movement-owned cursor. */
    static RouteProjection forwardRouteProjection(xin.bbtt.physics.WaypointFlightTarget route, Vector3d position) {
        var points = route.waypoints();
        RouteProjection best = null;
        double distance = Double.POSITIVE_INFINITY;
        for (int i = Math.max(1, route.index()); i < points.size(); i++) {
            var start = points.get(i - 1);
            var delta = new Vector3d(points.get(i)).sub(start);
            double length = delta.length();
            if (length < 1e-8) continue;
            var direction = new Vector3d(delta).div(length);
            double along = Math.max(0, Math.min(length, new Vector3d(position).sub(start).dot(direction)));
            var point = new Vector3d(start).fma(along, direction);
            double candidateDistance = point.distanceSquared(position);
            if (candidateDistance < distance) {
                distance = candidateDistance;
                best = new RouteProjection(point, direction,
                        new Vector3d(start).fma(Math.min(length, along + 6), direction));
            }
        }
        return best;
    }

    private ElytraFlightController.Decision routeRejoinDecision(xin.bbtt.flight.FlightContext context, MovementSync sync) {
        if (emergencyLanding || lowRoute == null) return null;
        return lowRouteRejoin(context, lowRoute, flightConfiguration, recoveryContext(sync));
    }

    /** Geometry supplies an aim only. Admission requires live swept airborne and recovery proofs. */
    static ElytraFlightController.Decision lowRouteRejoin(xin.bbtt.flight.FlightContext context,
            xin.bbtt.physics.WaypointFlightTarget route, xin.bbtt.flight.FlightConfiguration configuration,
            xin.bbtt.physics.HazardRecovery.Context recovery) {
        var projection = forwardRouteProjection(route, context.state().position());
        if (projection == null) return null;
        var aim = projection.aim();
        float heading = (float) Math.toDegrees(Math.atan2(context.state().position().x - aim.x,
                aim.z - context.state().position().z));
        double bestCost = context.state().position().distanceSquared(aim);
        ElytraFlightController.Decision best = null;
        for (boolean rocket : new boolean[] {false, true}) {
            if (rocket && (recovery.attached() || recovery.cooldownTicks() > 0
                    || recovery.rocketFlightDurations().length == 0)) continue;
            for (float yaw : new float[] {heading, context.yaw(), heading - 45, heading + 45}) {
                for (float pitch : new float[] {-55, -30, -10, 0, 20, 40}) {
                    var candidate = new ElytraFlightController.Decision(yaw, pitch, rocket,
                            ElytraFlightController.Phase.AVOIDANCE);
                    var end = airbornePursuitPreview(context, candidate, configuration);
                    if (end == null || new Vector3d(end.position()).sub(context.state().position())
                            .dot(projection.direction()) < -1e-6) continue;
                    double cost = end.position().distanceSquared(aim);
                    if (cost >= bestCost || !lowDecisionSafe(context, candidate, configuration, recovery)) continue;
                    best = candidate;
                    bestCost = cost;
                }
            }
        }
        return best;
    }

    private boolean lowPursuit() {
        return lowAltitude && !emergencyLanding
                && target instanceof xin.bbtt.physics.EntityFlightTarget entity && !entity.gliding();
    }

    /** A safe landing label is not an airborne proof; validate the actual swept trajectory. */
    static ElytraFlightController.Decision retainAirbornePursuit(xin.bbtt.flight.FlightContext context,
            ElytraFlightController.Decision selected, Vector3d liveAim,
            xin.bbtt.flight.FlightConfiguration configuration) {
        return airbornePursuitPreview(context, selected, configuration) != null
                ? selected : localPursuitDecision(context, liveAim, configuration);
    }

    /** Bounded local pursuit when a rolling corridor is absent or exhausted. Never a landing request. */
    static ElytraFlightController.Decision localPursuitDecision(xin.bbtt.flight.FlightContext context,
            Vector3d liveAim, xin.bbtt.flight.FlightConfiguration configuration) {
        float heading = (float) Math.toDegrees(Math.atan2(context.state().position().x - liveAim.x,
                liveAim.z - context.state().position().z));
        ElytraFlightController.Decision best = null;
        double bestCost = Double.POSITIVE_INFINITY;
        for (boolean rocket : new boolean[] {false, true}) {
            if (rocket && (context.attached() || context.fireworks() == 0)) continue;
            for (float yaw : new float[] {heading, context.yaw(), heading - 45, heading + 45,
                    heading - 90, heading + 90, heading + 180}) {
                for (float pitch : new float[] {-55, -30, -10, 0, 20, 40}) {
                    var candidate = new ElytraFlightController.Decision(yaw, pitch, rocket,
                            ElytraFlightController.Phase.AVOIDANCE);
                    var end = airbornePursuitPreview(context, candidate, configuration);
                    if (end == null) continue;
                    // Horizontal pursuit must not reward descending onto a ground target.
                    double cost = Math.hypot(end.position().x - liveAim.x, end.position().z - liveAim.z)
                            + Math.max(0, context.state().position().y - end.position().y) * 2
                            + (rocket ? 2 : 0) + Math.abs(pitch - context.pitch()) * .002;
                    if (cost < bestCost) { bestCost = cost; best = candidate; }
                }
            }
        }
        return best;
    }

    /** Exact six-tick body sweep plus six-tick airborne continuation, including loaded hazards. */
    static PhysicsState airbornePursuitPreview(xin.bbtt.flight.FlightContext context,
            ElytraFlightController.Decision decision, xin.bbtt.flight.FlightConfiguration configuration) {
        var action = new xin.bbtt.flight.FlightAction.Glide(decision.yaw(), decision.pitch(), decision.firework());
        var physics = new xin.bbtt.physics.GlidingPhysics(action.yaw(), action.pitch(),
                context.attached() || action.firework(), xin.bbtt.physics.PlayerInput.NONE, context.movement());
        var state = context.state();
        if (state.onGround()) return null;
        for (int tick = 0; tick < 12; tick++) {
            var result = physics.step(context.world(), state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            if (result.onGround() || result.collidedX() || result.collidedZ()
                    || !xin.bbtt.physics.PublicFlightBridge.known(context, state, result)) return null;
            var from = state.position();
            var to = result.position();
            if (context.world().isBoxColliding(Math.min(from.x, to.x) - PlayerPose.HALF_WIDTH,
                    Math.min(from.y, to.y), Math.min(from.z, to.z) - PlayerPose.HALF_WIDTH,
                    Math.max(from.x, to.x) + PlayerPose.HALF_WIDTH,
                    Math.max(from.y, to.y) + PlayerPose.GLIDING.height(),
                    Math.max(from.z, to.z) + PlayerPose.HALF_WIDTH)) return null;
            // Water ends vanilla glide even though it causes no damage.
            for (int x = (int) Math.floor(Math.min(from.x, to.x) - PlayerPose.HALF_WIDTH);
                    x <= (int) Math.floor(Math.max(from.x, to.x) + PlayerPose.HALF_WIDTH); x++) {
                for (int y = (int) Math.floor(Math.min(from.y, to.y));
                        y <= (int) Math.floor(Math.max(from.y, to.y) + PlayerPose.GLIDING.height()); y++) {
                    for (int z = (int) Math.floor(Math.min(from.z, to.z) - PlayerPose.HALF_WIDTH);
                            z <= (int) Math.floor(Math.max(from.z, to.z) + PlayerPose.HALF_WIDTH); z++) {
                        String block = context.world().getBlockStateAt(new Vector3d(x, y, z)).blockName();
                        if (block.equals("water") || block.equals("minecraft:water")
                                || block.equals("bubble_column") || block.equals("minecraft:bubble_column")) return null;
                    }
                }
            }
            if (xin.bbtt.physics.PublicFlightBridge.hazard(context, state, result, true).raw() > 0
                    || xin.bbtt.physics.PublicFlightBridge.hazard(context, state, result, false).raw() > 0) return null;
            for (var rule : configuration.damageRules()) {
                var damage = rule.damage(context, action, state, result);
                if (!Double.isFinite(damage.raw()) || !Double.isFinite(damage.reduced())
                        || damage.raw() > 0 || damage.reduced() > 0) return null;
            }
            state = new PhysicsState(to, result.velocity(), false, result.fallDistance(), result.pose());
        }
        return state;
    }

    private void updateLowRoute(MovementSync sync, PhysicsState state) {
        if (flightConfiguration.modes().stream().noneMatch(mode -> mode instanceof xin.bbtt.flight.VanillaGlideMode)) return;
        boolean required = target instanceof xin.bbtt.physics.EntityFlightTarget entity ? !entity.gliding()
                : xin.bbtt.listeners.RegistryDataListener.hasCeiling()
                && !xin.bbtt.physics.CeilingCollision.isAboveRoof(sync.getWorld(), state.position());
        if (!required) {
            if (target instanceof xin.bbtt.physics.EntityFlightTarget entity && entity.gliding()) {
                lowAltitude = false;
                lowRoute = null;
                if (lowPlan != null) lowPlan.cancel(false);
                lowPlan = null;
            }
            if (!lowAltitude) return;
        }
        lowAltitude = true;
        if (emergencyLanding && !lowEmergencyRoute) {
            lowRoute = null;
            if (lowPlan != null) lowPlan.cancel(false);
            lowPlan = null;
            lowEmergencyRoute = true;
            lowPlanTicks = 0;
        }
        boolean pursuit = !emergencyLanding && target instanceof xin.bbtt.physics.EntityFlightTarget;
        Vector3d endpoint = pursuit ? xin.bbtt.physics.WaypointFlightTarget.pursuitAim(target, state) : target.aim(state);
        if (lowPlan != null && lowPlan.isDone()) {
            var result = lowPlan.getNow(null);
            lowPlan = null;
            runIfCurrent(() -> {
                if (isFinished()) return;
                // Legacy waypoint lists have no corridor certificate to reconcile;
                // planner-backed routes must bind their cursor to the actual body.
                if (result != null && (emergencyLanding || pursuit || result.destination().distanceSquared(endpoint) <= 16)
                        && (!result.route().terrainValidated() || result.route().reconcile(sync.getWorld(), state))) {
                    lowRoute = result.route();
                    lowRouteAcceptedTick = flightTicks;
                    if (lowRoute.pursuit()) lowRoute.follow(target, result.destination());
                }
            });
        }
        if (lowPlan == null && !state.onGround() && sync.isGliding()
                && (lowPlanTicks++ % 20 == 0 || !pursuit && !emergencyLanding && lowRoute != null && lowRoute.goal().distanceSquared(endpoint) > 16)) {
            lowPlan = emergencyLanding ? LowAltitudeFlightPlan.emergency(sync.getWorld(), state, sync.getMovementParameters(), recoveryContext(sync))
                    : pursuit ? LowAltitudeFlightPlan.pursuit(sync.getWorld(), state, endpoint, sync.getMovementParameters(), true, recoveryContext(sync))
                    : LowAltitudeFlightPlan.submit(sync.getWorld(), state, endpoint, sync.getMovementParameters(), true, recoveryContext(sync));
        }
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

    static boolean hazardousGroundHandoff(MovementSync sync) {
        var p = sync.position.get();
        for (int x = -1; x <= 1; x++) for (int z = -1; z <= 1; z++) {
            if (harmBelow(sync.getWorld(), new PhysicsState(new Vector3d(p).add(x, 0, z),
                    new Vector3d(), false))) return true;
        }
        return false;
    }

    /** Nongliding safety continuation; proof work stays outside the navigation lock. */
    private void tickNonglidingLanding(MovementSync sync) {
        boolean stopped = GroundedFlightStop.stopped(sync);
        var command = stopped ? null : xin.bbtt.physics.GroundHazardExit.prove(sync.getWorld(),
                new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get(),
                        sync.fallDistance.get()), sync.getMovementParameters());
        runIfCurrent(() -> {
            if (isFinished() || sync.isGliding()) return;
            if (stopped && GroundedFlightStop.stopped(sync)) {
                sync.clearInput();
                setFinished(true);
                return;
            }
            if (command != null) {
                sync.clearInput();
                sync.yaw.set(command.yaw());
                sync.setInput(command.input());
            } else {
                // Missing proof is not permission to abandon an airborne body.
                GroundedFlightStop.hold(sync);
            }
        });
    }

    private void finish(boolean arrived) {
        // A rejected steering candidate is not a physical stop for cancellation.
        if (independentLanding && !GroundedFlightStop.stopped(MovementSync.INSTANCE)) return;
        if (MovementSync.INSTANCE.onGround.get() && hazardousGroundHandoff(MovementSync.INSTANCE))
            GroundedFlightStop.hold(MovementSync.INSTANCE);
        completedAboveRoof = arrived && targetBelowRoof;
        setFinished(true);
        if (arrived) {
            MovementSync.getLogger().info(LangManager.get("movementsync.command.elytra.fly.arrived"));
            MovementSync.getLogger().info("Flight leg state: position={}, onGround={}, gliding={}; navigation arrival remains subject to supported 3D completion",
                    MovementSync.INSTANCE.position.get(), MovementSync.INSTANCE.onGround.get(), MovementSync.INSTANCE.isGliding());
        } else {
            MovementSync.getLogger().info(LangManager.get("movementsync.command.elytra.fly.failed"));
        }
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
        MovementSync.INSTANCE.revokeFlightControls(this);
        // Cancelling steering must not fabricate a landing or switch off airborne physics.
        if (MovementSync.INSTANCE.onGround.get() && hazardousGroundHandoff(MovementSync.INSTANCE))
            GroundedFlightStop.hold(MovementSync.INSTANCE);
        setFinished(true);
        if (lowPlan != null) lowPlan.cancel(false);
        if (transition != null) {
            var active = transition;
            transition = null;
            active.movement().onStop();
        }
    }
}
