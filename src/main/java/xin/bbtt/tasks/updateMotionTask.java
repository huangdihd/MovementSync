package xin.bbtt.tasks;

import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerPosPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerPosRotPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerRotPacket;
import org.joml.Vector3d;
import org.joml.Vector3i;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.Server;

import static xin.bbtt.MovementSync.gravitationalAcceleration;
import static xin.bbtt.MovementSync.terminalVelocity;

public class updateMotionTask implements Runnable {
    private static final double PLAYER_HALF_WIDTH = 0.299;
    private static final double MAX_STEP_HEIGHT =
        xin.bbtt.pathfinding.StandablePositionResolver.MAX_STEP_RISE;
    private static final double STEP_EPSILON = 1.0e-7;
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

    private boolean isPlayerBoxColliding(Vector3d pos) {
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

        return MovementSync.INSTANCE.getWorld().isBoxColliding(
            pos.x - halfWidth, pos.y, pos.z - halfWidth,
            pos.x + halfWidth, pos.y + height, pos.z + halfWidth);
    }

    /**
     * Applies vanilla-style automatic stepping for grounded horizontal motion.
     * The candidate support height comes from the exact block-state collision
     * boxes, so path blocks, slabs, stairs, trapdoors, and directional shapes
     * keep their real heights instead of being treated as whole cubes.
     */
    private boolean tryStepUp(
            Vector3d position,
            double candidateX,
            double candidateZ,
            double tickStartY) {
        if (!MovementSync.INSTANCE.onGround.get() || MovementSync.INSTANCE.isRiding()) return false;

        java.util.OptionalDouble support = MovementSync.INSTANCE.getWorld().findHighestCollisionTop(
            candidateX - PLAYER_HALF_WIDTH, position.y + STEP_EPSILON, candidateZ - PLAYER_HALF_WIDTH,
            candidateX + PLAYER_HALF_WIDTH, tickStartY + MAX_STEP_HEIGHT, candidateZ + PLAYER_HALF_WIDTH
        );
        if (support.isEmpty()) return false;

        Vector3d stepped = new Vector3d(candidateX, support.getAsDouble(), candidateZ);
        if (isPlayerBoxColliding(stepped)) return false;
        position.set(stepped);
        return true;
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
            syncPositionToServer(false, true, false);
            return;
        }

        Vector3d position = new Vector3d(MovementSync.INSTANCE.position.get());
        Vector3i currentChunk = MovementSync.INSTANCE.getWorld().getChunk(position);
        if (!MovementSync.INSTANCE.getWorld().chunkLoaded(currentChunk.x, currentChunk.z)) return;

        Vector3d velocity = new Vector3d(MovementSync.INSTANCE.velocity.get());
        final double initialVelY = velocity.y;
        Vector3d displacement = new Vector3d();

        checkOnGround();
        if (MovementSync.INSTANCE.onGround.get() && velocity.y < 0) {
            velocity.y = 0;
        }

        final double tickStartY = position.y;

        /*
         * Match the client tick order: move with the velocity that was already
         * accumulated for this tick, then apply gravity and vertical drag for
         * the next tick.  Applying gravity before the move shortens the first
         * jump tick by about 0.087 blocks and makes every jump trajectory
         * diverge from the server's movement prediction.
         */
        displacement.x = velocity.x;
        displacement.z = velocity.z;
        boolean groundedAtTickStart = MovementSync.INSTANCE.onGround.get();
        boolean hasUpwardImpulse = velocity.y > 0;
        if (!groundedAtTickStart || hasUpwardImpulse) {
            displacement.y = velocity.y;
            velocity.y = Math.max(
                    (velocity.y + gravitationalAcceleration.y) * MovementSync.verticalDrag,
                    terminalVelocity);
        } else {
            displacement.y = 0;
            if (velocity.y < 0) velocity.y = 0;
        }

        boolean collidedX = false;
        boolean collidedZ = false;

        if (displacement.x != 0) {
            Vector3d testPosX = new Vector3d(position.x + displacement.x, position.y, position.z);
            if (isPlayerBoxColliding(testPosX)) {
                if (!tryStepUp(position, testPosX.x, testPosX.z, tickStartY)) {
                    displacement.x = 0;
                    velocity.x = 0;
                    collidedX = true;
                }
            } else {
                position.x += displacement.x;
            }
        }

        if (displacement.z != 0) {
            Vector3d testPosZ = new Vector3d(position.x, position.y, position.z + displacement.z);
            if (isPlayerBoxColliding(testPosZ)) {
                if (!tryStepUp(position, testPosZ.x, testPosZ.z, tickStartY)) {
                    displacement.z = 0;
                    velocity.z = 0;
                    collidedZ = true;
                }
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

        if (displacement.y < 0) {
            double desiredY = position.y + displacement.y;
            double resolvedY = VerticalCollisionResolver.resolveDownwardY(
                MovementSync.INSTANCE.getWorld(), position, 0.299, displacement.y);
            position.y = resolvedY;
            if (resolvedY > desiredY + 1.0e-9) velocity.y = 0;
        } else {
            position.y += displacement.y;
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