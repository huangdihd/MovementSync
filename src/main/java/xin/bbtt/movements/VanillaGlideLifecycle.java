package xin.bbtt.movements;

import org.joml.Vector3d;
import org.joml.Vector3i;
import xin.bbtt.MovementSync;
import xin.bbtt.inventory.InventoryManager;
import xin.bbtt.physics.ElytraDurability;
import xin.bbtt.physics.ElytraFlightController;
import xin.bbtt.physics.FallPrediction;
import xin.bbtt.physics.PhysicsState;

/** Internal vanilla glide lifecycle; callers retain navigation guards and leg orchestration. */
final class VanillaGlideLifecycle {
    private int takeoffTicks;
    private int rocketCooldown;
    private long flightTicks;
    private long rocketRequestTick;
    private long rocketRequestPhysicsTick;
    private long rocketRequestFrameTick;
    private long requestedBoostTicks;
    private boolean rocketAttachmentConfirmed;
    private boolean airborne;
    private boolean lowAltitudeTakeoff;
    private float launchYaw;
    private float launchPitch;
    private String launchFailure;
    private boolean deploymentRequested;

    void allowLowAltitudeTakeoff(float yaw, float pitch) {
        lowAltitudeTakeoff = true;
        launchYaw = yaw;
        launchPitch = pitch;
    }

    private boolean provenDeployment(MovementSync sync) {
        return lowAltitudeTakeoff && LowAltitudeFlightPlan.deployment(sync.getWorld(),
                new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get(), sync.fallDistance.get()),
                sync.getMovementParameters(), launchYaw, launchPitch) != null;
    }

    /** Called inside the movement's current-request guard. */
    void init(MovementSync sync) {
        // Remember an inherited glide before physics can land it between action ticks.
        sync.getLandingMemory().takeoff(sync.getWorld(), sync.position.get());
        sync.getLandingMemory().observe(sync.getWorld(), sync.position.get());
        airborne = sync.isGliding();
        if (!airborne) sync.clearInput();
        if (!airborne && lowAltitudeTakeoff) {
            sync.setYawToward(launchYaw);
            sync.pitch.set(launchPitch);
        }
        if (!airborne && !lowAltitudeTakeoff && sync.onGround.get() && hazardousLaunchMargin(sync)) {
            launchFailure = "hazardous launch requires a validated pre-jump route";
            logTakeoff(sync, "blocked-before-jump: " + launchFailure);
            return;
        }
        if (!airborne && (lowAltitudeTakeoff || hazardousLaunchMargin(sync))) {
            var state = new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get(), sync.fallDistance.get());
            var abort = xin.bbtt.physics.DeploymentSafety.assess(sync.getWorld(), state,
                    sync.getMovementParameters(), lowAltitudeTakeoff ? launchYaw : sync.yaw.get(),
                    lowAltitudeTakeoff ? launchPitch : sync.pitch.get());
            if (!abort.safe() || lowAltitudeTakeoff && !provenDeployment(sync)) {
                launchFailure = !abort.safe() ? abort.reason() : "deployment preview rejected";
                logTakeoff(sync, "blocked-before-jump: " + launchFailure);
                return;
            }
        }
        if (!airborne && sync.onGround.get() && (!underCeiling(sync) || provenDeployment(sync))) {
            logTakeoff(sync, "jump-requested");
            sync.jump();
        }
    }

    private static boolean hazardousLaunchMargin(MovementSync sync) {
        var p = sync.position.get();
        for (int x = -1; x <= 1; x++) for (int z = -1; z <= 1; z++) {
            var probe = new Vector3d(p).add(x, 0, z);
            if (ElytraFlyMovement.harmBelow(sync.getWorld(),
                    new PhysicsState(probe, new Vector3d(), false))) return true;
        }
        return false;
    }

    boolean awaitingTakeoff(MovementSync sync) {
        if (airborne && !sync.isGliding() && !sync.onGround.get()) {
            logTakeoff(sync, deploymentRequested ? "glide-lost-after-start-request (server denial or interruption)"
                    : "inherited-glide-lost");
            airborne = false;
        }
        return !airborne && !sync.isGliding();
    }

    /** Flight legs retry deployment; the standalone takeoff movement remains one-shot. */
    void tickTakeoff(MovementSync sync, Runnable fail) {
        // Revalidate at deployment: a preview can become stale while waiting for descent.
        if (launchFailure != null) {
            fail.run();
            return;
        }
        if (++takeoffTicks > 60 || sync.isRiding()
                || !sync.isGliding() && underCeiling(sync) && !lowAltitudeTakeoff && !eligibleForAirborneTakeoff(sync)) {
            logTakeoff(sync, takeoffTicks > 60 ? "timeout" : sync.isRiding() ? "riding" : "ceiling-rescue-rejected");
            fail.run();
            return;
        }
        if (!sync.onGround.get() && sync.velocity.get().y <= 0) {
            var abort = lowAltitudeTakeoff ? xin.bbtt.physics.DeploymentSafety.assess(sync.getWorld(),
                    new PhysicsState(sync.position.get(), sync.velocity.get(), false, sync.fallDistance.get()),
                    sync.getMovementParameters(), launchYaw, launchPitch) : null;
            if (lowAltitudeTakeoff && (!provenDeployment(sync) || !abort.safe())) {
                logTakeoff(sync, "deployment-revalidation-rejected" + (abort.safe() ? "" : ": " + abort.reason()));
                fail.run();
                return;
            }
            airborne = sync.startGliding();
            if (!deploymentRequested || !airborne) logTakeoff(sync, airborne
                    ? "start-request-sent (not server acceptance)" : "start-request-rejected");
            deploymentRequested |= airborne;
        } else if (takeoffTicks == 1 || takeoffTicks % 20 == 0) {
            logTakeoff(sync, "waiting-for-descent");
        }
    }

    private void logTakeoff(MovementSync sync, String reason) {
        var frame = sync.getPlacementFrame();
        sync.getLogger().info("Glide lifecycle: {} actionTick={} physicsTick={} low={} pos={} velocity={} ground={} gliding={} riding={} yaw={} pitch={}",
                reason, takeoffTicks, frame == null ? "unobserved" : frame.tick(), lowAltitudeTakeoff, sync.position.get(), sync.velocity.get(),
                sync.onGround.get(), sync.isGliding(), sync.isRiding(), sync.yaw.get(), sync.pitch.get());
    }

    void markAirborne() {
        airborne = true;
    }

    void tickCooldown(MovementSync sync) {
        flightTicks++;
        // Preserve the established cruise budget: expensive planning must not make
        // rockets spend faster than one per forty decisions. MovementSync's shared
        // request latch independently enforces forty committed physical ticks across
        // lifecycle and owner handoffs.
        if (rocketCooldown > 0) rocketCooldown--;
        confirmedBoostTicks(sync, sync.hasAttachedFirework());
    }

    /** Queued use is not thrust. Only a subsequently observed attachment confirms it. */
    long confirmedBoostTicks(MovementSync sync, boolean attached) {
        if (!attached) {
            if (rocketAttachmentConfirmed) {
                requestedBoostTicks = 0;
                rocketAttachmentConfirmed = false;
            }
            return 0;
        }
        if (requestedBoostTicks == 0) return 1; // Inherited/unknown rocket: this tick only.
        rocketAttachmentConfirmed = true;
        long physical = sync.physicsTickVersion();
        var frame = sync.getPlacementFrame();
        long elapsed = physical == 0 ? Math.max(flightTicks - rocketRequestTick,
                frame == null ? 0 : frame.tick() - rocketRequestFrameTick)
                : Math.max(0, physical - rocketRequestPhysicsTick);
        // Charge all request-to-confirmation latency; never restart the lifetime on observation.
        return Math.max(1, requestedBoostTicks - elapsed);
    }

    int rocketCooldown() {
        return rocketCooldown;
    }

    /** Called after prediction, inside the same current-request and finished guards as before. */
    void execute(MovementSync sync, ElytraFlightController.Decision decision,
                 InventoryManager.ElytraAction equipmentAction) {
        sync.setYawToward(decision.yaw());
        sync.pitch.set(decision.pitch());
        if (decision.firework() && rocketCooldown == 0 && !sync.hasAttachedFirework()
                && equipmentAction != InventoryManager.ElytraAction.WAITING) {
            // A rejected use request provides no thrust and must not suppress rescue retries.
            // useFirework selects the first hotbar rocket, not necessarily the held slot.
            int[] flights = sync.inventoryManager.getHotbarFireworkFlightDurations();
            if (sync.useFirework()) {
                rocketCooldown = 40;
                rocketRequestTick = flightTicks;
                rocketRequestPhysicsTick = sync.physicsTickVersion();
                var frame = sync.getPlacementFrame();
                rocketRequestFrameTick = frame == null ? 0 : frame.tick();
                // Vanilla minimum lifeTime plus its expiry boost tick (see GlideRange).
                requestedBoostTicks = flights.length == 0 ? 0 : 10L * (1 + Math.max(0L, flights[0])) + 1;
                rocketAttachmentConfirmed = false;
            }
        }
    }

    static InventoryManager.ElytraAction prepareFlight(MovementSync sync) {
        return sync.getInventoryManager().prepareElytra(sync.onGround.get(), sync::queueBeforeMovement);
    }

    static boolean hasFlightBudget(MovementSync sync, Vector3i goal) {
        Vector3d position = sync.position.get();
        return sync.getInventoryManager().canStartFlight(
                Math.hypot(position.x - (goal.x + 0.5), position.z - (goal.z + 0.5)));
    }

    static boolean shouldFly(double distance, boolean elytra, boolean fireworks,
                             boolean walk, boolean gliding) {
        return !walk && elytra && (gliding || distance > TravelMovement.FLIGHT_DISTANCE_THRESHOLD && fireworks);
    }

    static boolean shouldFly(double distance, ElytraDurability durability,
                             boolean fireworks, boolean walk, boolean gliding) {
        return durability.canStart(distance) && shouldFly(distance, durability.equipped(), fireworks, walk, gliding);
    }

    /** Evaluated only for a non-environmental airborne travel leg. */
    static boolean eligibleForAirborneTakeoff(MovementSync sync) {
        return sync.hasEquippedElytra()
                && sync.inventoryManager.getElytraDurability().usable()
                && FallPrediction.predict(sync.getWorld(),
                    new PhysicsState(sync.position.get(), sync.velocity.get(),
                            false, sync.fallDistance.get()), sync.getMovementParameters()).dangerous()
                // Under a roof, deploy only while descending and when the glide itself is proven safe.
                && (!underCeiling(sync) || !sync.onGround.get() && sync.velocity.get().y <= 0 && safeRescue(sync));
    }

    /** A ceiling dimension alone is not a veto when the launch column really is open. */
    private static boolean underCeiling(MovementSync sync) {
        return xin.bbtt.listeners.RegistryDataListener.hasCeiling()
                && !TravelMovement.isOpen(sync.getWorld(), sync.position.get());
    }

    /** Use the deployed controls and shared glide prediction, recovery and damage rules. */
    private static boolean safeRescue(MovementSync sync) {
        var state = new PhysicsState(sync.position.get(), sync.velocity.get(), false, sync.fallDistance.get());
        var context = new xin.bbtt.flight.FlightContext(sync.getWorld(), state, sync.getMovementParameters(),
                xin.bbtt.physics.DamageParameters.fromPlayer(sync.inventoryManager,
                        sync.getMovementParameterState().effectsSnapshot()),
                sync.getMovementParameterState().health(), state.position(), sync.yaw.get(), sync.pitch.get(),
                0, sync.hasAttachedFirework(), 1, true, java.util.Map.of());
        var mode = xin.bbtt.flight.VanillaGlideMode.INSTANCE;
        var action = new xin.bbtt.flight.FlightAction.Glide(context.yaw(), context.pitch(), false);
        var selector = new xin.bbtt.flight.FlightSelector(
                new xin.bbtt.flight.FlightContextBuilder().addStandard().build());
        return selector.safety(context, mode, action, mode.predict(context, action)).damageClass()
                == xin.bbtt.flight.FlightSelector.DamageClass.SAFE;
    }
}
