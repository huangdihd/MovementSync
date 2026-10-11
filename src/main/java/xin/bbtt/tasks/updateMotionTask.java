package xin.bbtt.tasks;

import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerPosPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerPosRotPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerRotPacket;
import org.joml.Vector3d;
import org.joml.Vector3i;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.Server;

import xin.bbtt.physics.PhysicsResult;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PhysicsStepper;
import xin.bbtt.physics.WalkingPhysics;

public class updateMotionTask implements Runnable {
    private static final double PLAYER_HALF_WIDTH = 0.299;
    private static final PhysicsStepper WALKING_PHYSICS = new WalkingPhysics();

    private record BodyDimensions(double halfWidth, double height) {}
    Vector3d lastPos = new Vector3d();
    float lastPitch = 0;
    float lastYaw = 0;
    boolean lastHorizontalCollision = false;

    public void syncPositionToServer(
            boolean horizontalCollision, boolean positionChanged, boolean rotationChanged) {
        boolean onGround = MovementSync.INSTANCE.onGround.get();
        Vector3d position = MovementSync.INSTANCE.position.get();
        float yaw = MovementSync.INSTANCE.yaw.get();
        float pitch = MovementSync.INSTANCE.pitch.get();

        if (positionChanged && rotationChanged) {
            Bot.INSTANCE.getSession().send(new ServerboundMovePlayerPosRotPacket(
                    onGround, horizontalCollision,
                    position.x, position.y, position.z, yaw, pitch));
        } else if (positionChanged) {
            Bot.INSTANCE.getSession().send(new ServerboundMovePlayerPosPacket(
                    onGround, horizontalCollision,
                    position.x, position.y, position.z));
        } else if (rotationChanged) {
            Bot.INSTANCE.getSession().send(new ServerboundMovePlayerRotPacket(
                    onGround, horizontalCollision, yaw, pitch));
        } else {
            // A collision-state transition still needs a movement packet.
            Bot.INSTANCE.getSession().send(new ServerboundMovePlayerPosPacket(
                    onGround, horizontalCollision,
                    position.x, position.y, position.z));
        }
    }

    public static void checkOnGround() {
        Vector3d position = new Vector3d(MovementSync.INSTANCE.position.get());
        MovementSync.INSTANCE.onGround.set(MovementSync.INSTANCE.getWorld().isOnGround(position));
    }

    private BodyDimensions getBodyDimensions() {
        double halfWidth = PLAYER_HALF_WIDTH; // Player width is 0.6 blocks.
        double height = 1.8;

        if (MovementSync.INSTANCE.isRiding()) {
            xin.bbtt.Entity.Entity vehicle = MovementSync.INSTANCE.getWorld().getEntity(MovementSync.INSTANCE.getVehicleId());
            if (vehicle != null) {
                halfWidth = vehicle.getWidth() / 2.0;
                height = vehicle.getHeight();
            }
        } else {
            Object pose = MovementSync.INSTANCE.getSelfMetadata().get("pose");
            if (pose != null) {
                String poseName = pose.toString();
                if (poseName.contains("SNEAKING")) {
                    height = 1.5;
                } else if (poseName.contains("SWIMMING") || poseName.contains("FALL_FLYING")) {
                    height = 0.6;
                }
            }
        }

        return new BodyDimensions(halfWidth, height);
    }

    private boolean isPlayerBoxColliding(Vector3d pos) {
        BodyDimensions dimensions = getBodyDimensions();
        return MovementSync.INSTANCE.getWorld().isBoxColliding(
            pos.x - dimensions.halfWidth(), pos.y, pos.z - dimensions.halfWidth(),
            pos.x + dimensions.halfWidth(), pos.y + dimensions.height(), pos.z + dimensions.halfWidth());
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
            MovementSync.INSTANCE.applyPersistentGazeIfIdle();
            syncPositionToServer(false, true, true);
            return;
        }

        Vector3d position = new Vector3d(MovementSync.INSTANCE.position.get());
        Vector3i currentChunk = MovementSync.INSTANCE.getWorld().getChunk(position);
        if (!MovementSync.INSTANCE.getWorld().chunkLoaded(currentChunk.x, currentChunk.z)) return;

        Vector3d velocity = new Vector3d(MovementSync.INSTANCE.velocity.get());
        final double initialVelY = velocity.y;
        checkOnGround();
        BodyDimensions dimensions = getBodyDimensions();
        PhysicsResult result = WALKING_PHYSICS.step(
            MovementSync.INSTANCE.getWorld(),
            new PhysicsState(position, velocity, MovementSync.INSTANCE.onGround.get()),
            dimensions.halfWidth(), dimensions.height());
        position = result.position();
        velocity = result.velocity();
        boolean collidedX = result.collidedX();
        boolean collidedZ = result.collidedZ();

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
        MovementSync.INSTANCE.applyPersistentGazeIfIdle();

        boolean horizontalCollision = collidedX || collidedZ;
        boolean positionChanged = !lastPos.equals(position);
        boolean rotationChanged = lastPitch != MovementSync.INSTANCE.pitch.get()
                || lastYaw != MovementSync.INSTANCE.yaw.get();
        if (positionChanged || rotationChanged || lastHorizontalCollision != horizontalCollision) {
            checkOnGround();
            syncPositionToServer(horizontalCollision, positionChanged, rotationChanged);
            lastPos.set(position);
            lastPitch = MovementSync.INSTANCE.pitch.get();
            lastYaw = MovementSync.INSTANCE.yaw.get();
            lastHorizontalCollision = horizontalCollision;
        }
    }
}