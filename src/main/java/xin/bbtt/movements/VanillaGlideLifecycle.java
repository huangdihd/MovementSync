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
    private boolean airborne;

    /** Called inside the movement's current-request guard. */
    void init(MovementSync sync) {
        // Remember an inherited glide before physics can land it between action ticks.
        sync.getLandingMemory().takeoff(sync.getWorld(), sync.position.get());
        sync.getLandingMemory().observe(sync.getWorld(), sync.position.get());
        airborne = sync.isGliding();
        if (!airborne && sync.onGround.get()) sync.jump();
    }

    boolean awaitingTakeoff(MovementSync sync) {
        return !airborne && !sync.isGliding();
    }

    /** Flight legs retry deployment; the standalone takeoff movement remains one-shot. */
    void tickTakeoff(MovementSync sync, Runnable fail) {
        if (++takeoffTicks > 60 || sync.isRiding()) {
            fail.run();
            return;
        }
        if (!sync.onGround.get() && sync.velocity.get().y <= 0) {
            airborne = sync.startGliding();
        }
    }

    void markAirborne() {
        airborne = true;
    }

    void tickCooldown() {
        if (rocketCooldown > 0) rocketCooldown--;
    }

    int rocketCooldown() {
        return rocketCooldown;
    }

    /** Called after prediction, inside the same current-request and finished guards as before. */
    void execute(MovementSync sync, ElytraFlightController.Decision decision,
                 InventoryManager.ElytraAction equipmentAction) {
        sync.setYawToward(decision.yaw());
        sync.pitch.set(decision.pitch());
        if (decision.firework() && !sync.hasAttachedFirework()
                && equipmentAction != InventoryManager.ElytraAction.WAITING) {
            // Latch before useFirework, including when inventory cannot complete the action.
            rocketCooldown = 40;
            sync.useFirework();
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
                            false, sync.fallDistance.get()), sync.getMovementParameters()).dangerous();
    }
}
