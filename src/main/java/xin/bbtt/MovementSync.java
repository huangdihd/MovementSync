package xin.bbtt;

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

    public int entityId = -1;
    public AtomicReference<Vector3d> position = new AtomicReference<>(new Vector3d(0, 0, 0));
    public AtomicReference<Vector3d> velocity = new AtomicReference<>(new Vector3d(0, 0, 0));
    public AtomicReference<Float> pitch = new AtomicReference<>(0f);
    public AtomicReference<Float> yaw = new AtomicReference<>(0f);
    public static final Vector3d gravitationalAcceleration = new Vector3d(0, -0.08, 0);
    public static final double terminalVelocity = -3.92;
    public static final double movementSpeed = 0.2159;
    public static final double gapJumpSpeedMultiplier = 1.3;
    public static final double jumpVelocity = 0.42;
    public static final double verticalDrag = 0.9800000190734863D;
    public AtomicBoolean onGround = new AtomicBoolean(true);
    private ScheduledExecutorService physicalSimulationService;
    public ScheduledExecutorService movementService;

    @Getter
    private final Map<String, Object> selfMetadata = new java.util.concurrent.ConcurrentHashMap<>();

    @Getter @Setter
    private int vehicleId = -1;
    
    @Setter @Getter
    private org.joml.Vector3i activeGoal = null;
    
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
        position.set(new Vector3d(0, 0, 0));
        velocity.set(new Vector3d(0, 0, 0));
        pitch.set(0f);
        yaw.set(0f);

        Bot.INSTANCE.addPacketListener(new TeleportPacketListener(), this);
        Bot.INSTANCE.addPacketListener(new LoginPacketListener(), this);
        Bot.INSTANCE.addPacketListener(new EntityIdRecorder(), this);
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
        physicalSimulationService.shutdown();
        movementService.shutdown();
    }

    public void jump() {
        if (!onGround.get()) return;
        velocity.updateAndGet(v -> new Vector3d(v).add(0, jumpVelocity, 0));
        onGround.set(false);
    }

    public void lookAt(Vector3d target) {
        getMovementController().addMovement(new xin.bbtt.movements.LookAtMovement(target));
    }

    public void directLookAt(Vector3d target) {
        Vector3d headPos = getHeadPosition();
        Vector3d diff = new Vector3d(target).sub(headPos);
        double distanceXZ = Math.sqrt(diff.x * diff.x + diff.z * diff.z);
        float targetYaw = (float) Math.toDegrees(Math.atan2(-diff.x, diff.z));
        float targetPitch = (float) Math.toDegrees(Math.atan2(-diff.y, distanceXZ));
        this.yaw.set(targetYaw);
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
        planAndPublish(claimNavigationRequest(goal, -1, -1, allowDigging));
    }

    public long beginStaticNavigationRequest(org.joml.Vector3i goal) {
        return claimNavigationRequest(goal, -1, -1, false).generation();
    }

    public void startFollowingNavigation(int entityId, double keepDistance, boolean allowDigging) {
        planAndPublish(claimNavigationRequest(null, entityId, keepDistance, allowDigging));
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
                if (isNavigationRequestCurrent(request.generation())) activeGoal = null;
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
            long generation = navigationGeneration.incrementAndGet();
            movementController.cancelAll();
            return generation;
        }
    }

    public void cancelNavigation() {
        synchronized (navigationLock) {
            navigationGeneration.incrementAndGet();
            movementController.cancelAll();
            activeGoal = null;
            followTargetId = -1;
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
        }
    }

    public Vector3d getHeadPosition() {
        return new Vector3d(MovementSync.INSTANCE.position.get()).add(Direction.UP.getVector(1.62));
    }

    public boolean isRiding() {
        return vehicleId != -1;
    }
}
