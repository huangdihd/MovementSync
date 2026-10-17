package xin.bbtt;

import xin.bbtt.physics.PlayerInput;
import org.geysermc.mcprotocollib.network.packet.Packet;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.Queue;
import java.util.Objects;

import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundPlayerCommandPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PlayerState;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.Hand;
import xin.bbtt.inventory.ItemRegistry;
import java.util.OptionalInt;
import lombok.Getter;
import lombok.Setter;
import org.joml.Vector3d;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import xin.bbtt.inventory.InventoryManager;
import xin.bbtt.listeners.*;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.mcbot.plugin.Plugin;
import xin.bbtt.movement.MovementController;
import xin.bbtt.tasks.updateMotionTask;
import xin.bbtt.world.Direction;
import xin.bbtt.world.World;

import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class MovementSync implements Plugin {
    @Getter
    private static final Logger logger = LoggerFactory.getLogger(MovementSync.class.getSimpleName());
    public static MovementSync INSTANCE;

    @Getter
    private final xin.bbtt.physics.MovementParameterState movementParameterState =
            new xin.bbtt.physics.MovementParameterState();

    public xin.bbtt.physics.MovementParameters getMovementParameters() {
        return movementParameterState.snapshot();
    }

    public int entityId = -1;
    public AtomicReference<Vector3d> position = new AtomicReference<>(new Vector3d(0, 0, 0));
    public AtomicReference<Vector3d> velocity = new AtomicReference<>(new Vector3d(0, 0, 0));
    private final AtomicReference<PlayerInput> input =
            new AtomicReference<>(PlayerInput.NONE);
    private final AtomicBoolean jumpRequested = new AtomicBoolean();
    private final Queue<Packet> beforeMovementPackets =
            new ConcurrentLinkedQueue<>();

    public PlayerInput getInput() {
        return input.get();
    }

    public void setInput(PlayerInput keys) {
        input.set(Objects.requireNonNull(keys, "keys"));
    }

    public void clearInput() {
        input.set(PlayerInput.NONE);
        jumpRequested.set(false);
    }

    /** Consumes only the one-shot request; held jump remains held. */
    public PlayerInput snapshotInput() {
        if (inventoryManager.isInventorySwapOpen()) return PlayerInput.NONE;
        PlayerInput keys = input.get();
        return jumpRequested.getAndSet(false) ? keys.withJump(true) : keys;
    }

    /** Last completed physics tick, containing only the position/rotation reported to the server. */
    public record PlacementFrame(long tick, double x, double y, double z,
                                 float yaw, float pitch, double eyeHeight) {}
    private volatile PlacementFrame placementFrame;

    public PlacementFrame getPlacementFrame() { return placementFrame; }

    public void publishPlacementFrame(double x, double y, double z, float yaw, float pitch, double eyeHeight) {
        PlacementFrame previous = placementFrame;
        placementFrame = new PlacementFrame(previous == null ? 1 : previous.tick() + 1,
                x, y, z, yaw, pitch, eyeHeight);
    }

    public void queueBeforeMovement(Packet packet) {
        beforeMovementPackets.add(Objects.requireNonNull(packet, "packet"));
    }

    public Packet pollBeforeMovementPacket() {
        return beforeMovementPackets.poll();
    }

    public AtomicReference<Float> pitch = new AtomicReference<>(0f);
    public AtomicReference<Float> yaw = new AtomicReference<>(0f);
    public static final Vector3d gravitationalAcceleration = new Vector3d(0, -0.08, 0);
    public static final double terminalVelocity = -3.92;
    public static final double movementSpeed = 0.2159;
    public static final double gapJumpSpeedMultiplier = 1.3;
    public static final double jumpVelocity = 0.42;
    public static final double verticalDrag = 0.9800000190734863D;
    public AtomicBoolean onGround = new AtomicBoolean(true);
    public final AtomicReference<Double> fallDistance = new AtomicReference<>(0.0);
    private final AtomicBoolean gliding = new AtomicBoolean();
    private ScheduledExecutorService physicalSimulationService;
    public ScheduledExecutorService movementService;

    @Getter
    private final Map<String, Object> selfMetadata = new java.util.concurrent.ConcurrentHashMap<>();

    @Getter @Setter
    private int vehicleId = -1;
    
    @Getter
    private org.joml.Vector3i activeGoal = null;

    @Getter
    private final xin.bbtt.recovery.StaticGotoRecovery gotoRecovery =
            new xin.bbtt.recovery.StaticGotoRecovery();
    @Getter
    private final xin.bbtt.movements.LandingMemory landingMemory = new xin.bbtt.movements.LandingMemory();
    @Getter
    private volatile String recoveryDimension;

    public void setRecoveryDimension(String dimension) {
        landingMemory.dimension(dimension);
        recoveryDimension = dimension;
    }

    public void setActiveGoal(org.joml.Vector3i goal) {
        synchronized (navigationLock) {
            activeGoal = goal;
            if (goal == null) clearRecoverableGoto("arrived");
        }
    }

    private void clearRecoverableGoto(String reason) {
        if (gotoRecovery.getTask() != null) getLogger().info("Static goto cleared: {}", reason);
        gotoRecovery.clear();
    }

    /** Session reset: invalidate paths, but retain the static goto intent. */
    public void suspendNavigationForRecovery() {
        synchronized (navigationLock) {
            gotoRecovery.suspend();
            landingMemory.clear();
            inventoryManager.resetSession();
            reconnectAirRecovery.reset();
            if (gotoRecovery.isPending()) getLogger().info("Static goto suspended for reconnect");
            navigationGeneration.incrementAndGet();
            movementController.cancelAll();
            activeGoal = null;
            followTargetId = -1;
            clearInput();
        }
    }

    /** Called after the authoritative first position has been applied. */
    public void onRecoveryTeleport() {
        if (gotoRecovery.onTeleport()) reconnectAirRecovery.positioned();
        else if (!gotoRecovery.isPending()) cancelNavigation();
    }

    @Getter
    private final xin.bbtt.physics.ReconnectAirRecovery reconnectAirRecovery =
            new xin.bbtt.physics.ReconnectAirRecovery();

    /** Tick integration supplies whether the bot is in Game, not merely connected. */
    public void tickNavigationRecovery(boolean game) {
        gotoRecovery.tick();
        if (gotoRecovery.expire()) getLogger().info("Static goto expired after 30 minutes");
        // The physics thread calls this every tick: wait for the navigation lock only to resume.
        if (!gotoRecovery.isPending()) return;
        synchronized (navigationLock) {
            var task = gotoRecovery.claim(game, recoveryDimension);
            if (task == null) return;
            getLogger().info("Resuming static goto {}, {}, {} (walk={})", task.x(), task.y(), task.z(), task.walk());
            long generation = getNavigationGeneration();
            Runnable resume = () -> {
                if (isNavigationRequestCurrent(generation)) new xin.bbtt.commands.GotoCommandExecutor().resume(task);
            };
            if (movementService == null) resume.run();
            else movementService.execute(resume);
        }
    }

    public long beginGotoNavigationRequest(org.joml.Vector3i goal, boolean walk, boolean recovering) {
        synchronized (navigationLock) {
            if (!recovering) {
                clearRecoverableGoto("replaced by goto");
                gotoRecovery.remember(goal, walk, recoveryDimension);
                getLogger().info("Static goto remembered: {} (walk={})", goal, walk);
            }
            return claimNavigationRequest(goal, -1, -1, false).generation();
        }
    }
    
    @Getter @Setter
    private int followTargetId = -1;

    @Getter @Setter
    private float ridingSideways = 0;
    @Getter @Setter
    private float ridingForward = 0;
    @Getter @Setter
    private boolean ridingJump = false;


    @Getter @Setter
    private boolean ridingSneak = false;

    @Getter
    public final World world = new World();
    @Getter
    public final MovementController movementController = new MovementController();
    private final java.util.concurrent.atomic.AtomicLong navigationGeneration =
            new java.util.concurrent.atomic.AtomicLong();
    private final Object navigationLock = new Object();
    private final java.util.concurrent.atomic.AtomicReference<GazeTarget> gazeTarget =
            new java.util.concurrent.atomic.AtomicReference<>();
    @Getter
    public final InventoryManager inventoryManager = new InventoryManager();

    public MovementSync() {
        INSTANCE = this;
    }

    @Override
    public void onLoad() {
        LangManager.initLang(getClass().getClassLoader());
        try {
            LangManager.loadFromStream(getClass().getResourceAsStream("/zh_cn.lang"));
            LangManager.loadFromStream(getClass().getResourceAsStream("/en_us.lang"));
        } catch (Exception e) {
            // Lang not yet loaded, so log in plain text rather than via a lang key.
            getLogger().warn("Failed to load language files; falling back to message keys", e);
        }
        
        getLogger().info(LangManager.get("movementsync.plugin.loading"));
    }

    @Override
    public void onUnload() {
        getLogger().info(LangManager.get("movementsync.plugin.unloading"));
    }

    @Override
    public void onEnable() {
        getLogger().info(LangManager.get("movementsync.plugin.enabling"));
        suspendNavigationForRecovery();
        recoveryDimension = null;
        movementParameterState.reset();
        position.set(new Vector3d(0, 0, 0));
        velocity.set(new Vector3d(0, 0, 0));
        pitch.set(0f);
        yaw.set(0f);

        Bot.INSTANCE.addPacketListener(new TeleportPacketListener(), this);
        Bot.INSTANCE.addPacketListener(new PingPacketListener(), this);
        Bot.INSTANCE.addPacketListener(new VelocityPacketListener(), this);
        Bot.INSTANCE.addPacketListener(new LoginPacketListener(), this);
        Bot.INSTANCE.addPacketListener(new EntityIdRecorder(), this);
        Bot.INSTANCE.addPacketListener(new MovementParametersPacketListener(), this);
        Bot.INSTANCE.addPacketListener(new RespawnPacketListener(), this);
        Bot.INSTANCE.addPacketListener(new ChunkDataListener(), this);
        Bot.INSTANCE.addPacketListener(new RegistryDataListener(), this);
        Bot.INSTANCE.addPacketListener(new InventoryPacketListener(), this);

        Bot.INSTANCE.getPluginManager().events().registerEvents(new ServerChangeListener(),  this);
        Bot.INSTANCE.getPluginManager().events().registerEvents(new EntityPacketListener(), this);

        physicalSimulationService = Executors.newScheduledThreadPool(1);
        physicalSimulationService.scheduleAtFixedRate(new updateMotionTask(), 0, 50, TimeUnit.MILLISECONDS);
        movementService = Executors.newScheduledThreadPool(1);
    }

    @Override
    public void onDisable() {
        getLogger().info(LangManager.get("movementsync.plugin.disabling"));
        movementParameterState.reset();
        suspendNavigationForRecovery();
        if (physicalSimulationService != null) physicalSimulationService.shutdown();
        if (movementService != null) movementService.shutdown();
    }

    public boolean isGliding() {
        return gliding.get();
    }

    /** Updates local state, including authoritative server corrections; sends no packet. */
    public void setGliding(boolean gliding) {
        this.gliding.set(gliding);
    }

    /** Returns whether a start request was sent, not whether the server accepted it. */
    public synchronized boolean startGliding() {
        if (isGliding() || onGround.get() || isRiding() || entityId < 0) return false;
        if (!inventoryManager.canStartFlight(0)) return false;
        if (Bot.INSTANCE == null || Bot.INSTANCE.getSession() == null || !Bot.INSTANCE.getSession().isConnected()) return false;
        jump();
        setGliding(true);
        queueBeforeMovement(new ServerboundPlayerCommandPacket(
                entityId, PlayerState.START_ELYTRA_FLYING));
        return true;
    }

    /** The protocol has a start command only: stopping is local, never a toggle packet. */
    public boolean stopGliding() {
        boolean wasGliding = isGliding();
        setGliding(false);
        return wasGliding;
    }

    /** Selects a hotbar rocket; acceleration waits for server-confirmed entity attachment. */
    public synchronized boolean useFirework() {
        if (!isGliding()) return false;
        int rocketSlot = hotbarFireworks().firstSlot();
        if (rocketSlot < 0) return false;
        if (Bot.INSTANCE == null || Bot.INSTANCE.getSession() == null || !Bot.INSTANCE.getSession().isConnected()) return false;
        if (inventoryManager.getHeldSlot() != rocketSlot) {
            queueBeforeMovement(new org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundSetCarriedItemPacket(rocketSlot));
            inventoryManager.setHeldSlot(rocketSlot);
        }
        queueBeforeMovement(new ServerboundUseItemPacket(
                Hand.MAIN_HAND, Bot.INSTANCE.getAndIncreaseSequence(), yaw.get(), pitch.get()));
        return true;
    }

    public boolean hasEquippedElytra() {
        ItemStack[] inventory = inventoryManager.getInventory();
        return inventory != null && inventory.length > 6 && isItem(inventory[6], "elytra");
    }

    public int countHotbarFireworks() {
        return hotbarFireworks().count();
    }

    private record HotbarFireworks(int firstSlot, int count) {}

    private HotbarFireworks hotbarFireworks() {
        ItemStack[] inventory = inventoryManager.getInventory();
        int firstSlot = -1;
        int count = 0;
        if (inventory == null) return new HotbarFireworks(firstSlot, count);
        for (int slot = 36; slot < Math.min(45, inventory.length); slot++) {
            if (!isItem(inventory[slot], "firework_rocket")) continue;
            if (firstSlot < 0) firstSlot = slot - 36;
            count += inventory[slot].getAmount();
        }
        return new HotbarFireworks(firstSlot, count);
    }

    private static boolean isItem(ItemStack stack, String name) {
        if (stack == null || stack.getAmount() <= 0) return false;
        ItemRegistry.ItemEntry item = ItemRegistry.Instance.getItem(stack.getId());
        return item != null && name.equals(item.getName());
    }

    public boolean hasAttachedFirework() {
        if (!isGliding() || entityId < 0) return false;
        for (xin.bbtt.Entity.Entity entity : world.getEntities().values()) {
            if (entity.getType() != org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType.FIREWORK_ROCKET) continue;
            Object target = entity.getMetadata().get("attached_to_target");
            if (target instanceof OptionalInt attached && attached.isPresent()
                    && attached.getAsInt() == entityId) return true;
        }
        return false;
    }

    public void clearFlightState() {
        setGliding(false);
        selfMetadata.remove("shared_flags");
        selfMetadata.remove("pose");
    }

    public void jump() {
        jumpRequested.set(true);
    }

    public void lookAt(Vector3d target) {
        getMovementController().addMovement(new xin.bbtt.movements.LookAtMovement(target));
    }

    /** Direction changes preserve accumulated turns; server teleports write yaw directly. */
    public void setYawToward(float direction) {
        yaw.updateAndGet(current -> nearestEquivalentYaw(current, direction));
    }

    public static float nearestEquivalentYaw(float current, float direction) {
        float difference = (direction - current) % 360.0f;
        if (difference >= 180.0f) difference -= 360.0f;
        if (difference < -180.0f) difference += 360.0f;
        return current + difference;
    }

    public void directLookAt(Vector3d target) {
        Vector3d headPos = getHeadPosition();
        Vector3d diff = new Vector3d(target).sub(headPos);
        double distanceXZ = Math.sqrt(diff.x * diff.x + diff.z * diff.z);
        float targetYaw = (float) Math.toDegrees(Math.atan2(-diff.x, diff.z));
        float targetPitch = (float) Math.toDegrees(Math.atan2(-diff.y, distanceXZ));
        setYawToward(targetYaw);
        this.pitch.set(targetPitch);
    }

    public void setBlockGazeTarget(org.joml.Vector3i block) {
        if (block == null) throw new IllegalArgumentException("block gaze target must not be null");
        gazeTarget.set(new GazeTarget(new org.joml.Vector3i(block), null));
    }

    public void setEntityGazeTarget(int entityId) {
        gazeTarget.set(new GazeTarget(null, entityId));
    }

    public void clearGazeTarget() {
        gazeTarget.set(null);
    }

    public String describeGazeTarget() {
        GazeTarget target = gazeTarget.get();
        if (target == null) return "none";
        if (target.block() != null) {
            org.joml.Vector3i block = target.block();
            return String.format("block=(%d,%d,%d)", block.x, block.y, block.z);
        }
        xin.bbtt.Entity.Entity entity = world.getEntity(target.entityId());
        if (entity == null || entity.getPosition() == null) {
            return "entity_id=" + target.entityId() + " status=unavailable";
        }
        Vector3d position = entity.getPosition();
        return String.format(
                "entity_id=%d position=(%.2f,%.2f,%.2f)",
                target.entityId(), position.x, position.y, position.z);
    }

    public void applyPersistentGazeIfIdle() {
        applyPersistentGaze(movementController.getCurrentMovement() != null);
    }

    void applyPersistentGaze(boolean orientationBusy) {
        if (orientationBusy) return;
        GazeTarget target = gazeTarget.get();
        if (target == null) return;
        Vector3d point;
        if (target.block() != null) {
            org.joml.Vector3i block = target.block();
            point = new Vector3d(block.x + 0.5, block.y + 0.5, block.z + 0.5);
        } else {
            xin.bbtt.Entity.Entity entity = world.getEntity(target.entityId());
            if (entity == null || entity.getPosition() == null) return;
            point = new Vector3d(entity.getPosition()).add(
                    0, Math.max(0.1, entity.getHeight() * 0.9), 0);
        }
        directLookAt(point);
    }

    private record GazeTarget(org.joml.Vector3i block, Integer entityId) {}

    public void triggerAutoRepath() {
        triggerAutoRepath(-1, false);
    }

    /**
     * @param followKeepDistance follow keep radius in blocks; non-positive uses the PathMovement default.
     */
    public void triggerAutoRepath(double followKeepDistance) {
        triggerAutoRepath(followKeepDistance, false);
    }

    public void triggerAutoRepath(boolean allowDigging) {
        triggerAutoRepath(-1, allowDigging);
    }

    /** Starts a new request from the currently selected static/follow target. */
    public void triggerAutoRepath(double followKeepDistance, boolean allowDigging) {
        NavigationRequest request;
        synchronized (navigationLock) {
            request = claimNavigationRequest(
                    activeGoal, followTargetId, followKeepDistance, allowDigging);
        }
        planAndPublish(request);
    }

    public void startStaticNavigation(org.joml.Vector3i goal, boolean allowDigging) {
        cancelNavigation(false);
        planAndPublish(claimNavigationRequest(goal, -1, -1, allowDigging));
    }

    public long beginStaticNavigationRequest(org.joml.Vector3i goal) {
        synchronized (navigationLock) {
            clearRecoverableGoto("replaced by static navigation");
            return claimNavigationRequest(goal, -1, -1, false).generation();
        }
    }

    public void startFollowingNavigation(int entityId, double keepDistance, boolean allowDigging) {
        synchronized (navigationLock) {
            cancelNavigation(false);
        }
        NavigationRequest request = claimNavigationRequest(null, entityId, keepDistance, allowDigging);
        synchronized (navigationLock) {
            if (isNavigationRequestCurrent(request.generation())) {
                movementController.addMovement(new xin.bbtt.movements.FollowMovement(
                        entityId, keepDistance, allowDigging, request.generation()));
            }
        }
    }

    private NavigationRequest claimNavigationRequest(
            org.joml.Vector3i goal,
            int requestedFollowTargetId,
            double followKeepDistance,
            boolean allowDigging) {
        synchronized (navigationLock) {
            long generation = navigationGeneration.incrementAndGet();
            movementController.cancelAll();
            activeGoal = goal == null ? null : new org.joml.Vector3i(goal);
            followTargetId = requestedFollowTargetId;
            return new NavigationRequest(
                    generation, activeGoal, requestedFollowTargetId, followKeepDistance, allowDigging);
        }
    }

    private void planAndPublish(NavigationRequest request) {
        org.joml.Vector3i targetNodePos = request.goal();
        if (request.followTargetId() != -1) {
            xin.bbtt.Entity.Entity entity = world.getEntity(request.followTargetId());
            if (entity != null) {
                Vector3d p = entity.getPosition();
                targetNodePos = new org.joml.Vector3i(
                        (int)Math.floor(p.x),
                        xin.bbtt.pathfinding.StandablePositionResolver.nodeY(p.y),
                        (int)Math.floor(p.z));
            }
        }
        if (targetNodePos == null) return;

        Vector3d currentPos = position.get();
        xin.bbtt.pathfinding.Node start = new xin.bbtt.pathfinding.Node(
                (int)Math.floor(currentPos.x),
                xin.bbtt.pathfinding.StandablePositionResolver.nodeY(currentPos.y),
                (int)Math.floor(currentPos.z));
        xin.bbtt.pathfinding.Node goalNode = new xin.bbtt.pathfinding.Node(
                targetNodePos.x, targetNodePos.y, targetNodePos.z);
        if (start.equals(goalNode) && request.followTargetId() == -1) {
            synchronized (navigationLock) {
                if (isNavigationRequestCurrent(request.generation())) setActiveGoal(null);
            }
            return;
        }

        xin.bbtt.pathfinding.DStarLite pathfinder = new xin.bbtt.pathfinding.DStarLite(
                start,
                goalNode,
                new xin.bbtt.pathfinding.DefaultPathfindingContext(getWorld(), request.allowDigging())
        );
        java.util.List<xin.bbtt.pathfinding.PathStep> path = pathfinder.findPath(2000);
        if (path.size() <= 1) return;

        xin.bbtt.movements.PathMovement movement = new xin.bbtt.movements.PathMovement(
                path,
                request.followKeepDistance(),
                request.allowDigging(),
                request.goal(),
                request.followTargetId(),
                request.generation()
        );
        synchronized (navigationLock) {
            if (!isNavigationRequestCurrent(request.generation())) return;
            movementController.addMovement(movement);
        }
    }

    /** Invalidates and stops every older navigation request before a new one starts. */
    public long beginNavigationRequest() {
        synchronized (navigationLock) {
            clearRecoverableGoto("replaced by navigation");
            long generation = navigationGeneration.incrementAndGet();
            movementController.cancelAll();
            return generation;
        }
    }

    /** Clears a failed request's goal without disturbing newer navigation or other state. */
    public void abandonNavigationRequest(long generation) {
        synchronized (navigationLock) {
            if (!isNavigationRequestCurrent(generation)) return;
            activeGoal = null;
        }
    }

    private xin.bbtt.movements.ElytraFlyMovement safetyLanding;

    public boolean runIfSafetyMovementCurrent(xin.bbtt.movements.ElytraFlyMovement movement, Runnable action) {
        synchronized (navigationLock) {
            if (safetyLanding != movement || movement.isFinished()) return false;
            action.run();
            return true;
        }
    }

    public void cancelNavigation() {
        cancelNavigation(true);
    }

    private void cancelNavigation(boolean land) {
        synchronized (navigationLock) {
            if (land && isGliding() && safetyLanding != null && !safetyLanding.isFinished()) {
                getLogger().info("正在安全落地,落地后停止");
                return;
            }
            clearRecoverableGoto("cancelled or died");
            navigationGeneration.incrementAndGet();
            movementController.cancelAll();
            activeGoal = null;
            followTargetId = -1;
            safetyLanding = null;
            if (land && isGliding()) {
                safetyLanding = new xin.bbtt.movements.ElytraFlyMovement(position.get()).forIndependentLanding();
                getLogger().info("Navigation cancelled: safe landing takeover; 正在安全落地,落地后停止");
                movementController.addMovement(safetyLanding);
            }
        }
    }

    public long getNavigationGeneration() {
        return navigationGeneration.get();
    }

    public boolean isNavigationRequestCurrent(long generation) {
        return navigationGeneration.get() == generation;
    }

    public boolean insertMovementIfNavigationRequestCurrent(
            long generation, xin.bbtt.movement.Movement movement) {
        synchronized (navigationLock) {
            if (!isNavigationRequestCurrent(generation)) return false;
            movementController.insertMovement(movement);
            return true;
        }
    }

    public boolean addMovementIfNavigationRequestCurrent(
            long generation, xin.bbtt.movement.Movement movement) {
        synchronized (navigationLock) {
            if (!isNavigationRequestCurrent(generation)) return false;
            movementController.addMovement(movement);
            return true;
        }
    }

    public boolean runIfNavigationRequestCurrent(long generation, Runnable action) {
        synchronized (navigationLock) {
            if (!isNavigationRequestCurrent(generation)) return false;
            action.run();
            return true;
        }
    }

    private record NavigationRequest(
            long generation,
            org.joml.Vector3i goal,
            int followTargetId,
            double followKeepDistance,
            boolean allowDigging) {}

    public void requestRepath() {
        xin.bbtt.movement.Movement current = movementController.getCurrentMovement();
        if (current instanceof xin.bbtt.movements.PathMovement pathMovement) {
            pathMovement.requestRepath();
        } else if (current instanceof xin.bbtt.movements.TravelMovement travelMovement) {
            travelMovement.requestRepath();
        }
    }

    /** A block changed at the given position; replan only if it lies along the current route. */
    public void requestRepath(int x, int y, int z) {
        xin.bbtt.movement.Movement current = movementController.getCurrentMovement();
        if (current instanceof xin.bbtt.movements.PathMovement pathMovement) {
            pathMovement.requestRepath(x, y, z);
        } else if (current instanceof xin.bbtt.movements.TravelMovement travelMovement) {
            travelMovement.requestRepath(x, y, z);
        }
    }

    public Vector3d getHeadPosition() {
        return new Vector3d(MovementSync.INSTANCE.position.get()).add(Direction.UP.getVector(1.62));
    }

    public boolean isRiding() {
        return vehicleId != -1;
    }
}
