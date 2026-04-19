package xin.bbtt;

import lombok.Getter;
import lombok.Setter;
import org.joml.Vector3d;
import xin.bbtt.commands.*;
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
    public static MovementSync Instance;
    public int entityId = -1;
    public AtomicReference<Vector3d> position = new AtomicReference<>(new Vector3d(0, 0, 0));
    public AtomicReference<Vector3d> velocity = new AtomicReference<>(new Vector3d(0, 0, 0));
    public AtomicReference<Float> pitch = new AtomicReference<>(0f);
    public AtomicReference<Float> yaw = new AtomicReference<>(0f);
    public static final Vector3d gravitationalAcceleration = new Vector3d(0, -0.08, 0);
    public static final double terminalVelocity = -3.92;
    public static final double movementSpeed = 0.2159;
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
    @Getter
    public final InventoryManager inventoryManager = new InventoryManager();

    public MovementSync() {
        Instance = this;
    }

    @Override
    public void onLoad() {
        LangManager.initLang(getClass().getClassLoader());
        try {
            LangManager.loadFromStream(getClass().getResourceAsStream("/zh_cn.lang"));
            LangManager.loadFromStream(getClass().getResourceAsStream("/en_us.lang"));
        } catch (Exception ignored) {}
        
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

        Bot.Instance.addPacketListener(new TeleportPacketListener(), this);
        Bot.Instance.addPacketListener(new EntityIdRecorder(), this);
        Bot.Instance.addPacketListener(new RespawnPacketListener(), this);
        Bot.Instance.addPacketListener(new ChunkDataListener(), this);
        Bot.Instance.addPacketListener(new RegistryDataListener(), this);
        Bot.Instance.addPacketListener(new InventoryPacketListener(), this);

        Bot.Instance.getPluginManager().registerCommand(new WhereAmICommand(), new WhereAmICommandExecutor(),  this);
        Bot.Instance.getPluginManager().registerCommand(new JumpCommand(), new JumpCommandExecutor(),  this);
        Bot.Instance.getPluginManager().registerCommand(new GetBlockAtCommand(), new GetBlockAtCommandExecutor(), this);
        Bot.Instance.getPluginManager().registerCommand(new WalkCommand(), new WalkCommandExecutor(), this);
        Bot.Instance.getPluginManager().registerCommand(new LookAtCommand(), new LookAtCommandExecutor(), this);
        Bot.Instance.getPluginManager().registerCommand(new MovementCommand(), new MovementCommandExecutor(), this);
        Bot.Instance.getPluginManager().registerCommand(new EntitiesCommand(), new EntitiesCommandExecutor(), this);
        Bot.Instance.getPluginManager().registerCommand(new InteractBlockCommand(), new InteractBlockCommandExecutor(), this);
        Bot.Instance.getPluginManager().registerCommand(new InteractEntityCommand(), new InteractEntityCommandExecutor(), this);
        Bot.Instance.getPluginManager().registerCommand(new InventoryCommand(), new InventoryCommandExecutor(), this);
        Bot.Instance.getPluginManager().registerCommand(new ContainerCommand(), new ContainerCommandExecutor(), this);
        Bot.Instance.getPluginManager().registerCommand(new IsPassableCommand(), new IsPassableCommandExecutor(), this);
        Bot.Instance.getPluginManager().registerCommand(new LookAtEntityCommand(), new LookAtEntityCommandExecutor(), this);
        Bot.Instance.getPluginManager().registerCommand(new GotoCommand(), new GotoCommandExecutor(), this);
        Bot.Instance.getPluginManager().registerCommand(new FollowCommand(), new FollowCommandExecutor(), this);
        Bot.Instance.getPluginManager().registerCommand(new RideCommand(), new RideCommandExecutor(), this);

        Bot.Instance.getPluginManager().events().registerEvents(new ServerChangeListener(),  this);
        Bot.Instance.getPluginManager().events().registerEvents(new EntityPacketListener(), this);

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
        velocity.updateAndGet(v -> new Vector3d(v).add(0, 0.42, 0));
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

    public void triggerAutoRepath() {
        org.joml.Vector3i targetNodePos = activeGoal;
        
        // If we are following an entity, override targetNodePos
        if (followTargetId != -1) {
            xin.bbtt.Entity.Entity entity = world.getEntity(followTargetId);
            if (entity != null) {
                Vector3d p = entity.getPosition();
                targetNodePos = new org.joml.Vector3i((int)Math.floor(p.x), (int)Math.floor(p.y), (int)Math.floor(p.z));
            }
        }

        if (targetNodePos == null) return;

        Vector3d currentPos = position.get();
        xin.bbtt.pathfinding.Node start = new xin.bbtt.pathfinding.Node((int)Math.floor(currentPos.x), (int)Math.floor(currentPos.y), (int)Math.floor(currentPos.z));
        xin.bbtt.pathfinding.Node goalNode = new xin.bbtt.pathfinding.Node(targetNodePos.x, targetNodePos.y, targetNodePos.z);

        if (start.equals(goalNode) && followTargetId == -1) {
            activeGoal = null;
            return;
        }

        // Repath limit to avoid too much calculation
        xin.bbtt.pathfinding.DStarLite pathfinder = new xin.bbtt.pathfinding.DStarLite(start, goalNode, getWorld());
        java.util.List<xin.bbtt.pathfinding.Node> path = pathfinder.findPath(2000);

        if (path.size() > 1) {
            getMovementController().cancelAll();
            getMovementController().addMovement(new xin.bbtt.movements.PathMovement(path));
        }
    }

    public void requestRepath() {
        xin.bbtt.movement.Movement current = movementController.getCurrentMovement();
        if (current instanceof xin.bbtt.movements.PathMovement pathMovement) {
            pathMovement.requestRepath();
        }
    }

    public Vector3d getHeadPosition() {
        return new Vector3d(MovementSync.Instance.position.get()).add(Direction.UP.getVector(1.62));
    }

    public boolean isRiding() {
        return vehicleId != -1;
    }
}
