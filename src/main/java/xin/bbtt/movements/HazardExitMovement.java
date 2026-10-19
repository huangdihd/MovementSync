package xin.bbtt.movements;

import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;
import xin.bbtt.physics.GroundHazardExit;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerInput;

/** Finite failure/cancel safety owner. Never resumes the old goal. */
public final class HazardExitMovement extends Movement {
    private final long generation;
    private int ticks;
    private PlayerInput owned;

    public HazardExitMovement(long generation) { this.generation = generation; }
    @Override public void init() {}
    @Override public long getTime() { return 8_000; }

    /** Local known contact, not unknown support, justifies an emergency owner. */
    public static boolean needed(MovementSync sync) {
        Object flags = sync.getSelfMetadata().get("shared_flags");
        boolean burning = flags instanceof Number number && (number.intValue() & 1) != 0;
        var position = sync.position.get();
        if (!position.isFinite()) return false;
        if (!xin.bbtt.physics.FlightHazards.safeLanding(sync.getWorld(), position)) return true;
        // A rejected/cleared glide can still coast off the shore after touchdown.
        // Use real nongliding physics, not residual ignition or unknown support.
        if (!sync.onGround.get()) return GroundHazardExit.airborneRisk(sync.getWorld(),
                new PhysicsState(position, sync.velocity.get(), false, sync.fallDistance.get()),
                sync.getMovementParameters());
        var velocity = sync.velocity.get();
        // Already outside contact and stationary: moving cannot extinguish fire.
        if (!velocity.isFinite() || velocity.x == 0 && velocity.z == 0) return false;
        // Fixed local probes bound work even for corrupt or unusually high velocity.
        var direction = new org.joml.Vector3d(velocity.x, 0, velocity.z).normalize();
        for (int distance = 1; distance <= 3; distance++) {
            if (!xin.bbtt.physics.FlightHazards.safeLanding(sync.getWorld(),
                    new org.joml.Vector3d(position).add(new org.joml.Vector3d(direction).mul(distance)))) return true;
        }
        return burning;
    }

    @Override public void onTick() {
        MovementSync sync = MovementSync.INSTANCE;
        if (!sync.isNavigationRequestCurrent(generation)) {
            finished = true;
            return;
        }
        // Physics proof must never hold the navigation lock: cancellation stays responsive.
        boolean stop = sync.isDead() || sync.isRiding() || sync.isGliding()
                || ++ticks > 120 || GroundedFlightStop.stopped(sync);
        var command = stop ? null : GroundHazardExit.prove(sync.getWorld(),
                new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get(),
                        sync.fallDistance.get()), sync.getMovementParameters());
        if (!sync.runIfNavigationRequestCurrent(generation, () -> {
            if (sync.isDead() || sync.isRiding() || sync.isGliding()) {
                release(sync);
                finished = true;
                return;
            }
            if (stop) {
                release(sync);
                finished = true;
                return;
            }
            sync.clearInput();
            if (command == null) {
                MovementSync.getLogger().warn("Hazard exit unproven; no safe finite walking stop found");
                finished = true;
                return;
            }
            sync.setYawToward(command.yaw());
            owned = command.input();
            sync.setInput(owned);
        })) finished = true;
    }

    private void release(MovementSync sync) {
        if (owned != null && sync.getInput() == owned) sync.clearInput();
        owned = null;
    }
    @Override public void onStop() { release(MovementSync.INSTANCE); }
    @Override public void onPause() { release(MovementSync.INSTANCE); }
}
