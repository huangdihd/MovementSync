package xin.bbtt.movements;

import java.util.ArrayList;
import xin.bbtt.MovementSync;
import xin.bbtt.flight.FlightAction;
import xin.bbtt.flight.FlightContext;
import xin.bbtt.flight.Trajectory;
import xin.bbtt.movement.Movement;
import xin.bbtt.physics.*;

/**
 * Internal cross-package adapter; plugins should use VanillaGlideMode, not this class.
 * Previews query the navigation thread's live equipment and world without changing controls.
 * Returned movements require a navigation guard before any MovementSync monitor is acquired.
 */
public final class VanillaTransitionBridge {
    private VanillaTransitionBridge() {}

    /** Returns a real-physics transition preview, or null when vanilla eligibility/search declines. */
    public static Trajectory preview(FlightContext c, boolean takeoff) {
        MovementSync sync = MovementSync.INSTANCE;
        if (sync == null || sync.isRiding()) return null;
        if (takeoff && !sync.isGliding()) {
            if (xin.bbtt.pathfinding.EnvironmentalTraversal.inWater(c.world(), c.state().position())
                    || xin.bbtt.pathfinding.EnvironmentalTraversal.climbing(c.world(), c.state().position())) return null;
            if (!c.state().onGround()) {
                if (!VanillaGlideLifecycle.eligibleForAirborneTakeoff(sync)) return null;
            } else {
                double distance = Math.hypot(c.target().x - c.state().position().x,
                        c.target().z - c.state().position().z);
                if (!VanillaGlideLifecycle.shouldFly(distance, sync.inventoryManager.getElytraDurability(),
                        c.fireworks() > 0, c.emergency(), false)
                        || !sync.inventoryManager.canStartFlight(distance)
                        || !TravelMovement.isOpen(c.world(), c.state().position())) return null;
            }
        } else if (!sync.isGliding()) return null;
        FlightAction.Glide action;
        if (takeoff) {
            action = new FlightAction.Glide(c.yaw(), c.pitch(), false);
        } else {
            // Use the same landing search, memory and rocket budget as execution,
            // rather than certifying an unrelated fixed-pitch landing.
            try {
                var decision = new ElytraFlightController(c.movement(), c.damage(), c.health())
                        .forceLand(c.world(), c.state(), c.yaw(), c.pitch(), c.attached(),
                                c.fireworks() > 0 ? sync.inventoryManager.getHotbarFireworkFlightDurations() : new int[0],
                                c.radius(), sync.getLandingMemory(), new LandingSearch());
                action = new FlightAction.Glide(decision.yaw(), decision.pitch(), decision.firework());
            } catch (IllegalStateException unavailable) {
                return null;
            }
        }
        var results = new ArrayList<PhysicsResult>();
        PhysicsState state = c.state();
        // Deployment waits for the descending half of the jump, exactly as the lifecycle does.
        if (takeoff && !sync.isGliding()) {
            for (int tick = 0; tick < 60 && (state.onGround() || state.velocity().y > 0); tick++) {
                var step = new WalkingPhysics(PlayerInput.NONE.withJump(tick == 0 && state.onGround()),
                        c.yaw(), true, c.movement()).step(c.world(), state, .3, 1.8);
                results.add(step);
                state = new PhysicsState(step.position(), step.velocity(), step.onGround(), step.fallDistance(), step.pose());
                if (tick > 0 && state.onGround()) return null;
            }
            if (state.onGround() || state.velocity().y > 0) return null;
        }
        var physics = new GlidingPhysics(action.yaw(), action.pitch(), c.attached() || action.firework(),
                c.movement().gravity(), c.movement().slowFalling());
        for (int tick = 0; tick < (takeoff ? 24 : 60); tick++) {
            var step = physics.step(c.world(), state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            results.add(step);
            state = new PhysicsState(step.position(), step.velocity(), step.onGround(), step.fallDistance(), step.pose());
            if (state.onGround()) break;
        }
        return new Trajectory(c.state(), results, action);
    }

    /** Creates an uninitialized deployment movement; preparation occurs only when ticked under the guard. */
    public static Movement takeoff() {
        return new Movement() {
            private final VanillaGlideLifecycle lifecycle = new VanillaGlideLifecycle();
            private boolean initialized;
            @Override public void init() {}
            @Override public void onTick() {
                if (finished) return;
                var sync = MovementSync.INSTANCE;
                if (!initialized) {
                    var preparation = VanillaGlideLifecycle.prepareFlight(sync);
                    if (preparation == xin.bbtt.inventory.InventoryManager.ElytraAction.WAITING) return;
                    if (preparation == xin.bbtt.inventory.InventoryManager.ElytraAction.LAND) { finished = true; return; }
                    lifecycle.init(sync);
                    initialized = true;
                }
                if (lifecycle.awaitingTakeoff(sync)) lifecycle.tickTakeoff(sync, () -> finished = true);
                else finished = true;
            }
            @Override public long getTime() { return -1; }
            @Override public void onStop() { finished = true; }
        };
    }

    /** Creates an uninitialized adaptive landing movement; caller owns navigation binding and lifecycle. */
    public static Movement landing(FlightContext c) {
        return new ElytraFlyMovement(c.target()).forSafeLanding();
    }
}
