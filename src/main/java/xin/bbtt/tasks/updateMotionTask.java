package xin.bbtt.tasks;

import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerPosRotPacket;
import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.Server;
import xin.bbtt.world.Direction;
import xin.bbtt.world.World;

import static xin.bbtt.MovementSync.gravitationalAcceleration;
import static xin.bbtt.MovementSync.terminalVelocity;

public class updateMotionTask implements Runnable {

    Vector3d lastPos = new Vector3d();
    float lastPitch = 0;
    float lastYaw = 0;

    public void syncPositionToServer() {
        boolean onGround = MovementSync.INSTANCE.onGround.get();
        Bot.INSTANCE.getSession().send(new ServerboundMovePlayerPosRotPacket(
                onGround,
                false,
                MovementSync.INSTANCE.position.get().x,
                MovementSync.INSTANCE.position.get().y,
                MovementSync.INSTANCE.position.get().z,
                MovementSync.INSTANCE.yaw.get(),
                MovementSync.INSTANCE.pitch.get()
        ));
    }

    public static void checkOnGround() {
        Vector3d position = new Vector3d(MovementSync.INSTANCE.position.get());
        MovementSync.INSTANCE.onGround.set(MovementSync.INSTANCE.getWorld().isOnGround(position));
    }

    private boolean isWallCollision(Vector3d pos) {
        return MovementSync.INSTANCE.getWorld().isSolid(pos);
    }

    private boolean isPlayerBoxColliding(Vector3d pos) {
        double w = 0.299; // Half width for player (0.6 / 2)
        double hMax = 1.8;

        if (MovementSync.INSTANCE.isRiding()) {
            xin.bbtt.Entity.Entity vehicle = MovementSync.INSTANCE.getWorld().getEntity(MovementSync.INSTANCE.getVehicleId());
            if (vehicle != null) {
                w = vehicle.getWidth() / 2.0;
                hMax = vehicle.getHeight();
            }
        } else {
            // Adjust player height based on pose
            Object pose = MovementSync.INSTANCE.getSelfMetadata().get("pose");
            if (pose != null) {
                String poseName = pose.toString();
                if (poseName.contains("SNEAKING")) {
                    hMax = 1.5;
                } else if (poseName.contains("SWIMMING") || poseName.contains("FALL_FLYING")) {
                    hMax = 0.6;
                }
            }
        }

        double[] heights = {0.0, hMax / 2.0, hMax - 0.01};

        for (double h : heights) {
            Vector3d[] corners = {
                    new Vector3d(pos.x - w, pos.y + h, pos.z - w),
                    new Vector3d(pos.x + w, pos.y + h, pos.z - w),
                    new Vector3d(pos.x - w, pos.y + h, pos.z + w),
                    new Vector3d(pos.x + w, pos.y + h, pos.z + w)
            };
            for (Vector3d corner : corners) {
                if (isWallCollision(corner)) return true;
            }
        }
        return false;
    }

    @Override
    public void run() {
        if (!Bot.INSTANCE.isRunning()) return;
        if (Bot.INSTANCE.getServer() != Server.Game) return;

        if (MovementSync.INSTANCE.isRiding()) {
            xin.bbtt.Entity.Entity vehicle = MovementSync.INSTANCE.getWorld().getEntity(MovementSync.INSTANCE.getVehicleId());
            if (vehicle != null) {
                MovementSync.INSTANCE.position.get().set(vehicle.getPosition());
                
                // Special handling for boats: need to paddle
                if (vehicle.getType().name().contains("BOAT")) {
                    float forward = MovementSync.INSTANCE.getRidingForward();
                    float sideways = MovementSync.INSTANCE.getRidingSideways();
                    boolean leftPaddle = forward > 0 || sideways > 0;
                    boolean rightPaddle = forward > 0 || sideways < 0;
                    if (leftPaddle || rightPaddle) {
                        Bot.INSTANCE.getSession().send(new org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.level.ServerboundPaddleBoatPacket(leftPaddle, rightPaddle));
                    }
                }
            }
            float sideways = MovementSync.INSTANCE.getRidingSideways();
            float forward = MovementSync.INSTANCE.getRidingForward();
            boolean jump = MovementSync.INSTANCE.isRidingJump();
            boolean sneak = MovementSync.INSTANCE.isRidingSneak();
            
            Bot.INSTANCE.getSession().send(new org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.level.ServerboundPlayerInputPacket(
                sideways > 0, sideways < 0, forward > 0, forward < 0, jump, sneak, false));
            syncPositionToServer();
            return;
        }

        Vector3d velocity = new Vector3d(MovementSync.INSTANCE.velocity.get());
        final double initialVelY = velocity.y;
        Vector3d displacement = new Vector3d();

        checkOnGround();
        if (MovementSync.INSTANCE.onGround.get() && velocity.y < 0) {
            velocity.y = 0;
        }

        Vector3d position = new Vector3d(MovementSync.INSTANCE.position.get());

        if (velocity.y > terminalVelocity) {
            if (!MovementSync.INSTANCE.onGround.get()) velocity.add(gravitationalAcceleration);
            velocity.y *= 0.9800000190734863D;
            displacement.add(velocity);
        } else if (velocity.y < 0) {
            velocity.y = terminalVelocity;
            displacement.add(new Vector3d(velocity).add(velocity).div(2));
        }

        boolean collidedX = false;
        boolean collidedZ = false;

        if (displacement.x != 0) {
            Vector3d testPosX = new Vector3d(position.x + displacement.x, position.y, position.z);
            if (isPlayerBoxColliding(testPosX)) {
                displacement.x = 0;
                velocity.x = 0;
                collidedX = true;
            } else {
                position.x += displacement.x;
            }
        }

        if (displacement.z != 0) {
            Vector3d testPosZ = new Vector3d(position.x, position.y, position.z + displacement.z);
            if (isPlayerBoxColliding(testPosZ)) {
                displacement.z = 0;
                velocity.z = 0;
                collidedZ = true;
            } else {
                position.z += displacement.z;
            }
        }

        if (displacement.y > 0) {
            Vector3d testPosY = new Vector3d(position.x, position.y + displacement.y, position.z);
            if (isPlayerBoxColliding(testPosY)) {
                displacement.y = 0;
                velocity.y = 0;
            }
        }

        Vector3d lowest = new Vector3d(position);
        lowest.y = Math.ceil(position.y);

        if (!MovementSync.INSTANCE.onGround.get()) {
            while (!MovementSync.INSTANCE.getWorld().isOnGround(lowest) && lowest.y > World.getMinWorldY()) {
                lowest.add(Direction.DOWN.getUnitVector());
            }
        }

        position.y += displacement.y;

        if (position.y < lowest.y){
            position.y = lowest.y;
            velocity.y = 0;
        }

        // Merge instead of set(): movements may change velocity concurrently during this
        // tick (e.g. WalkMovement.onStop subtracting its component); overwriting the whole
        // vector with our stale snapshot would silently undo those updates.
        final double newVelY = velocity.y;
        final boolean stopX = collidedX;
        final boolean stopZ = collidedZ;
        MovementSync.INSTANCE.velocity.updateAndGet(v -> {
            Vector3d merged = new Vector3d(v);
            if (merged.y == initialVelY) merged.y = newVelY;
            if (stopX) merged.x = 0;
            if (stopZ) merged.z = 0;
            return merged;
        });
        MovementSync.INSTANCE.position.set(position);

        if (!(lastPos.equals(position) && lastPitch == MovementSync.INSTANCE.pitch.get() && lastYaw == MovementSync.INSTANCE.yaw.get())) {
            checkOnGround();
            syncPositionToServer();
            lastPos.set(position);
            lastPitch = MovementSync.INSTANCE.pitch.get();
            lastYaw = MovementSync.INSTANCE.yaw.get();
        }
    }
}