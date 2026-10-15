package xin.bbtt.physics;

import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.movements.ElytraFlyMovement;
import xin.bbtt.world.World;

/** One-shot aerial safety check after a new play session has a position. */
public final class ReconnectAirRecovery {
    private boolean pending;
    private boolean positioned;
    private boolean warned;

    public void reset() {
        pending = true;
        positioned = false;
        warned = false;
    }

    public void positioned() {
        positioned = true;
    }

    public static boolean needsDeployment(boolean grounded, boolean gliding, double verticalSpeed,
            double heightAboveGround, boolean voidBelow) {
        return !grounded && !gliding && verticalSpeed < 0 && (voidBelow || heightAboveGround > 3);
    }

    public void tick(MovementSync sync, boolean game, boolean recoveringNavigation) {
        if (!pending || !positioned || !game) return;
        Vector3d p = sync.position.get();
        if (!sync.getWorld().chunkLoaded((int) Math.floor(p.x) >> 4, (int) Math.floor(p.z) >> 4)) return;
        if (sync.onGround.get()) {
            pending = false;
            return;
        }
        var ground = sync.getWorld().findHighestCollisionTop(p.x - .299, World.getMinWorldY(),
                p.z - .299, p.x + .299, p.y, p.z + .299);
        boolean deploy = needsDeployment(false, sync.isGliding(), sync.velocity.get().y,
                ground.isPresent() ? p.y - ground.getAsDouble() : Double.POSITIVE_INFINITY, ground.isEmpty());
        if (deploy) {
            if (!sync.inventoryManager.getElytraDurability().usable() || !sync.hasEquippedElytra()) {
                if (!warned) MovementSync.getLogger().warn("Reconnect aerial rescue unavailable: no usable elytra, durability={}",
                        sync.inventoryManager.getElytraDurability());
                warned = true;
                return;
            }
            if (sync.startGliding()) MovementSync.getLogger().info("Reconnect aerial rescue: requested glide at {}", p);
        }
        if (!sync.isGliding()) return;
        if (recoveringNavigation || sync.getMovementController().getCurrentMovement() != null) {
            pending = false;
            return;
        }
        try {
            Vector3d target = new ElytraFlightController().selectForcedLandingTarget(sync.getWorld(),
                    new PhysicsState(p, sync.velocity.get(), false, sync.fallDistance.get()),
                    sync.countHotbarFireworks(), sync.hasAttachedFirework());
            sync.getMovementController().addMovement(new ElytraFlyMovement(target));
            MovementSync.getLogger().info("Reconnect aerial rescue: no task; landing at {}", target);
            pending = false;
        } catch (IllegalStateException error) {
            if (!warned) MovementSync.getLogger().warn("Reconnect aerial rescue: {}", error.getMessage());
            warned = true;
        }
    }
}
