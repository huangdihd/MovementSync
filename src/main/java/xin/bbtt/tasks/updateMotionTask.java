package xin.bbtt.tasks;

import xin.bbtt.physics.PlayerInput;
import org.geysermc.mcprotocollib.network.packet.Packet;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.level.ServerboundPlayerInputPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundPlayerCommandPacket;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PlayerState;

import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerPosPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerPosRotPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerRotPacket;
import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.Bot;

import xin.bbtt.physics.GlidingPhysics;
import xin.bbtt.physics.PhysicsResult;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PhysicsStepper;
import xin.bbtt.physics.WalkingPhysics;

public class updateMotionTask implements Runnable {
    private static void sendIfConnected(Packet packet) {
        var session = Bot.INSTANCE.getSession();
        if (session != null && session.isConnected()) session.send(packet);
    }

    private static final double PLAYER_HALF_WIDTH = xin.bbtt.physics.PlayerPose.HALF_WIDTH;
    private PlayerInput lastInput = PlayerInput.NONE;
    private boolean lastSprinting;
    private int jumpCooldown;
    private xin.bbtt.physics.PlayerPose pose = xin.bbtt.physics.PlayerPose.STANDING;

    private record BodyDimensions(double halfWidth, double height) {}
    Vector3d lastPos = new Vector3d();
    float lastPitch = 0;
    float lastYaw = 0;
    boolean lastHorizontalCollision = false;
    boolean lastOnGround;
    private int ticksSincePositionPacket;

    public void syncPositionToServer(
            boolean horizontalCollision, boolean positionChanged, boolean rotationChanged) {
        syncPositionToServer(horizontalCollision, positionChanged, rotationChanged,
                MovementSync.INSTANCE.yaw.get(), MovementSync.INSTANCE.pitch.get());
    }

    private void syncPositionToServer(
            boolean horizontalCollision, boolean positionChanged, boolean rotationChanged,
            float yaw, float pitch) {
        boolean onGround = MovementSync.INSTANCE.onGround.get();
        Vector3d position = MovementSync.INSTANCE.position.get();

        if (positionChanged && rotationChanged) {
            sendIfConnected(new ServerboundMovePlayerPosRotPacket(
                    onGround, horizontalCollision,
                    position.x, position.y, position.z, yaw, pitch));
        } else if (positionChanged) {
            sendIfConnected(new ServerboundMovePlayerPosPacket(
                    onGround, horizontalCollision,
                    position.x, position.y, position.z));
        } else if (rotationChanged) {
            sendIfConnected(new ServerboundMovePlayerRotPacket(
                    onGround, horizontalCollision, yaw, pitch));
        } else {
            // A collision-state transition still needs a movement packet.
            sendIfConnected(new ServerboundMovePlayerPosPacket(
                    onGround, horizontalCollision,
                    position.x, position.y, position.z));
        }
    }

    public static void checkOnGround() {
        Vector3d position = new Vector3d(MovementSync.INSTANCE.position.get());
        MovementSync.INSTANCE.onGround.set(MovementSync.INSTANCE.getWorld().isOnGround(position));
    }

    private BodyDimensions getBodyDimensions() {
        double halfWidth = 0.299; // Preserve the unknown-vehicle riding fallback.
        double height = 1.8;

        if (MovementSync.INSTANCE.isRiding()) {
            xin.bbtt.Entity.Entity vehicle = MovementSync.INSTANCE.getWorld().getEntity(MovementSync.INSTANCE.getVehicleId());
            if (vehicle != null) {
                halfWidth = vehicle.getWidth() / 2.0;
                height = vehicle.getHeight();
            }
        } else if (MovementSync.INSTANCE.isGliding()) {
            // PlayerEntity's FALL_FLYING dimensions are floats, not an inset box.
            halfWidth = PLAYER_HALF_WIDTH;
            height = xin.bbtt.physics.PlayerPose.GLIDING.height();
        } else {
            halfWidth = PLAYER_HALF_WIDTH;
            // Collision and travel must use the same locally resolved pose.
            // Metadata may still describe standing while the client is crawling.
            height = pose.height();
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
        if (!Bot.INSTANCE.isRunning() || !MovementSync.INSTANCE.getGotoRecovery().hasPosition()) return;

        // A start request must not change flight physics or enqueue its action
        // between this tick's input snapshot and movement/tick-end packets.
        synchronized (MovementSync.INSTANCE) {
            try {
                tick();
            } catch (RuntimeException error) {
                // scheduleAtFixedRate silently cancels a task that throws; keep ticking.
                MovementSync.getLogger().error("Physics tick failed", error);
            } finally {
                // Close the same atomic packet batch, including on failure.
                sendIfConnected(
                    org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.ServerboundClientTickEndPacket.INSTANCE);
            }
        }
        // Lock order is navigation lock, then MovementSync: movement threads hold the navigation
        // lock while they start a glide, so goto recovery must not run under the tick monitor.
        try {
            MovementSync.INSTANCE.tickNavigationRecovery(Bot.INSTANCE.getServer() == xin.bbtt.mcbot.Server.Game);
        } catch (RuntimeException error) {
            MovementSync.getLogger().error("Goto recovery tick failed", error);
        }
    }

    private void tick() {
        boolean game = Bot.INSTANCE.getServer() == xin.bbtt.mcbot.Server.Game;
        MovementSync.INSTANCE.getReconnectAirRecovery().tick(MovementSync.INSTANCE, game,
                MovementSync.INSTANCE.getGotoRecovery().isPending());
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
                        sendIfConnected(new org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.level.ServerboundPaddleBoatPacket(leftPaddle, rightPaddle));
                    }
                }
            }
            float sideways = MovementSync.INSTANCE.getRidingSideways();
            float forward = MovementSync.INSTANCE.getRidingForward();
            boolean jump = MovementSync.INSTANCE.isRidingJump();
            boolean sneak = MovementSync.INSTANCE.isRidingSneak();
            
            sendIfConnected(new org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.level.ServerboundPlayerInputPacket(
                sideways > 0, sideways < 0, forward > 0, forward < 0, jump, sneak, false));
            MovementSync.INSTANCE.applyPersistentGazeIfIdle();
            syncPositionToServer(false, true, true);
            return;
        }

        Vector3d position = new Vector3d(MovementSync.INSTANCE.position.get());

        // Admit controls before observing rotation,
        // input or action packets. Embedded physics drivers share this boundary.
        PlayerInput input = MovementSync.INSTANCE.snapshotInput();
        xin.bbtt.physics.MovementParameters parameters = MovementSync.INSTANCE.getMovementParameters();
        if (input.sprint() && !parameters.canSprint()) {
            input = new PlayerInput(input.forward(), input.backward(), input.left(), input.right(),
                    input.jump(), input.sneak(), false);
        }
        // Never start swimming: normalize the request before both input/action packets
        // and travel, rather than predicting a standing body with a sprinting server state.
        BodyDimensions inputDimensions = getBodyDimensions();
        xin.bbtt.physics.EnvironmentalPhysics.Environment environment =
                xin.bbtt.physics.EnvironmentalPhysics.sample(MovementSync.INSTANCE.getWorld(), position,
                        inputDimensions.halfWidth(), inputDimensions.height());
        if (environment.waterHeight() > 0 || environment.lavaHeight() > 0
                || pose.shouldSlowDown(MovementSync.INSTANCE.isGliding(), environment.waterHeight() > 0, input.sneak())) {
            input = new PlayerInput(input.forward(), input.backward(), input.left(), input.right(),
                    input.jump(), input.sneak(), false);
        }
        float tickYaw = MovementSync.INSTANCE.yaw.get();
        float tickPitch = MovementSync.INSTANCE.pitch.get();
        // ElytraB checks the previous (released) input at the action, then
        // this tick's jump at movement. Deployment must precede the input edge.
        java.util.List<Packet> actions = new java.util.ArrayList<>();
        java.util.List<Packet> deferredActions = new java.util.ArrayList<>();
        boolean releaseBeforeDeployment = false;
        boolean deploying = false;
        Packet queuedAction;
        while ((queuedAction = MovementSync.INSTANCE.pollBeforeMovementPacket()) != null) {
            if (queuedAction instanceof ServerboundPlayerCommandPacket command
                    && command.getState() == PlayerState.START_ELYTRA_FLYING) {
                if (lastInput.jump()) {
                    releaseBeforeDeployment = true;
                    deferredActions.add(queuedAction);
                    MovementSync.INSTANCE.setGliding(false);
                    input = input.withJump(false);
                } else {
                    deploying = true;
                    MovementSync.INSTANCE.setGliding(true);
                    input = input.withJump(true);
                    sendIfConnected(queuedAction);
                }
            } else {
                actions.add(queuedAction);
            }
        }
        if (releaseBeforeDeployment) {
            deferredActions.addAll(actions);
            actions.clear();
            for (Packet action : deferredActions) MovementSync.INSTANCE.queueBeforeMovement(action);
        }
        if (!input.equals(lastInput)) {
            sendIfConnected(new ServerboundPlayerInputPacket(
                    input.forward(), input.backward(), input.left(), input.right(), input.jump(), input.sneak(), input.sprint()));
            lastInput = input;
        }
        if (input.sprint() != lastSprinting) {
            sendIfConnected(new ServerboundPlayerCommandPacket(
                    MovementSync.INSTANCE.entityId, input.sprint()
                    ? PlayerState.START_SPRINTING
                    : PlayerState.STOP_SPRINTING));
            lastSprinting = input.sprint();
        }
        for (Packet action : actions) {
            // Steering can change after a rocket is queued. Use the rotation of
            // the tick that actually sends it, not the request thread's snapshot.
            if (action instanceof org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket useItem) {
                action = useItem.withYRot(tickYaw).withXRot(tickPitch);
            }
            sendIfConnected(action);
        }
        Vector3d initialVelocity = MovementSync.INSTANCE.velocity.get();
        Vector3d velocity = new Vector3d(initialVelocity);
        // PlayerEntity.tick updates pose after travel: deployment still uses the standing box.
        BodyDimensions dimensions = deploying ? new BodyDimensions(PLAYER_HALF_WIDTH, (double) 1.8f) : getBodyDimensions();
        boolean gliding = MovementSync.INSTANCE.isGliding();
        if (jumpCooldown > 0) jumpCooldown--;
        if (!input.jump()) jumpCooldown = 0;
        boolean jumpAllowed = jumpCooldown == 0;
        if (!gliding && input.jump() && jumpAllowed && MovementSync.INSTANCE.onGround.get()) {
            jumpCooldown = 10;
        }
        PhysicsStepper physics = gliding
                ? new GlidingPhysics(tickYaw, tickPitch,
                        MovementSync.INSTANCE.hasAttachedFirework(), input, parameters)
                : new WalkingPhysics(input, tickYaw, jumpAllowed, parameters);
        PhysicsResult result = physics.step(
            MovementSync.INSTANCE.getWorld(),
            new PhysicsState(position, velocity, MovementSync.INSTANCE.onGround.get(),
                    parameters.slowFalling() || parameters.levitationAmplifier() >= 0
                            ? 0 : MovementSync.INSTANCE.fallDistance.get(), pose),
            dimensions.halfWidth(), dimensions.height());
        MovementSync.INSTANCE.fallDistance.set(result.fallDistance());
        position = result.position();
        velocity = result.velocity();
        boolean collidedX = result.collidedX();
        boolean collidedZ = result.collidedZ();

        if (gliding) {
            MovementSync.INSTANCE.velocity.set(velocity);
            MovementSync.INSTANCE.onGround.set(result.onGround());
            // Verified against Mojang 1.21.11 client + mappings: LivingEntity.updateFallFlying
            // (chl.fT, source lines 3215-3239) gates flag clearance on !level.isClientSide.
            // LocalPlayer's post-travel onGround check clears abilities.flying, NOT glide.
            // Keep flight travel until shared_flags metadata clears the gliding bit.
        } else {
            // Identity CAS preserves even a concurrent same-valued write by a legacy action.
            // If another writer wins, its complete velocity takes priority over this tick's drag.
            MovementSync.INSTANCE.velocity.compareAndSet(initialVelocity, velocity);
        }
        if (!gliding) MovementSync.INSTANCE.onGround.set(result.onGround());
        pose = pose.afterTravel(MovementSync.INSTANCE.getWorld(), position, gliding, false, input.sneak());
        MovementSync.INSTANCE.position.set(position);
        MovementSync.INSTANCE.applyPersistentGazeIfIdle();

        MovementSync.INSTANCE.completePhysicsStep();
        MovementSync.INSTANCE.recordFlightTick(new MovementSync.FlightTickTrace(
                new Vector3d(position), new Vector3d(velocity), result.onGround(), collidedX, collidedZ,
                gliding, MovementSync.INSTANCE.hasAttachedFirework(), tickYaw, tickPitch, input, result.fallDistance()));
        boolean horizontalCollision = collidedX || collidedZ;
        if (gliding && horizontalCollision && !lastHorizontalCollision) {
            MovementSync.getLogger().warn("Glide horizontal collision: position={}, velocity={}, collidedX={}, collidedZ={}, yaw={}, pitch={}, attachedRocket={}",
                    position, velocity, collidedX, collidedZ, tickYaw, tickPitch,
                    MovementSync.INSTANCE.hasAttachedFirework());
        }
        // ClientPlayerEntity.sendMovementPackets repeats the position every 20 ticks while
        // standing still; servers and anti-cheats use it to resolve pending corrections.
        boolean positionChanged = !lastPos.equals(position) || ++ticksSincePositionPacket >= 20;
        if (positionChanged) ticksSincePositionPacket = 0;
        // Report the rotation used for acceleration, not concurrent next-tick steering.
        float packetYaw = tickYaw;
        float packetPitch = tickPitch;
        boolean rotationChanged = lastPitch != packetPitch || lastYaw != packetYaw;
        if (positionChanged || rotationChanged || lastHorizontalCollision != horizontalCollision
                || lastOnGround != MovementSync.INSTANCE.onGround.get()) {
            syncPositionToServer(horizontalCollision, positionChanged, rotationChanged, packetYaw, packetPitch);
            lastPos.set(position);
            lastPitch = packetPitch;
            lastYaw = packetYaw;
            lastHorizontalCollision = horizontalCollision;
            lastOnGround = MovementSync.INSTANCE.onGround.get();
        }
        MovementSync.INSTANCE.publishPlacementFrame(lastPos.x, lastPos.y, lastPos.z, lastYaw, lastPitch,
                pose == xin.bbtt.physics.PlayerPose.CROUCHING ? (double) 1.27f
                        : pose == xin.bbtt.physics.PlayerPose.STANDING ? (double) 1.62f : (double) 0.4f);
    }
}