package xin.bbtt;

import lombok.Getter;
import org.joml.Vector3d;
import xin.bbtt.commands.*;
import xin.bbtt.listeners.*;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.mcbot.plugin.Plugin;
import xin.bbtt.movement.MovementController;
import xin.bbtt.movements.JumpMovement;
import xin.bbtt.movements.LookAtMovement;
import xin.bbtt.tasks.updateMotionTask;
import xin.bbtt.world.Direction;
import xin.bbtt.world.World;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class MovementSync implements Plugin {
    public static MovementSync Instance;
    public int entityId = -1;
    public AtomicReference<Vector3d> position = new AtomicReference<>();
    public AtomicReference<Vector3d> velocity = new AtomicReference<>();
    public AtomicReference<Float> pitch = new AtomicReference<>();
    public AtomicReference<Float> yaw = new AtomicReference<>();
    public static final Vector3d gravitationalAcceleration = new Vector3d(0, -0.08, 0);
    public static final double terminalVelocity = -3.92;
    public static final double movementSpeed = 0.2159;
    public AtomicBoolean onGround = new AtomicBoolean(true);
    private ScheduledExecutorService physicalSimulationService;
    public ScheduledExecutorService movementService;
    @Getter
    public final World world = new World();
    @Getter
    public final MovementController movementController = new MovementController();

    public MovementSync() {
        Instance = this;
    }

    @Override
    public void onLoad() {
        LangManager.initLang(getClass().getClassLoader());
        // Try multiple paths to ensure loading
        try {
            LangManager.loadFromStream(getClass().getResourceAsStream("/zh_cn.lang"));
            LangManager.loadFromStream(getClass().getResourceAsStream("/en_us.lang"));
            LangManager.loadFromStream(getClass().getResourceAsStream("/lang/zh_cn.lang"));
            LangManager.loadFromStream(getClass().getResourceAsStream("/lang/en_us.lang"));
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

        Bot.Instance.getPluginManager().registerCommand(new WhereAmICommand(), new WhereAmICommandExecutor(),  this);
        Bot.Instance.getPluginManager().registerCommand(new JumpCommand(), new JumpCommandExecutor(),  this);
        Bot.Instance.getPluginManager().registerCommand(new GetBlockAtCommand(), new GetBlockAtCommandExecutor(), this);
        Bot.Instance.getPluginManager().registerCommand(new WalkCommand(), new WalkCommandExecutor(), this);
        Bot.Instance.getPluginManager().registerCommand(new LookAtCommand(), new LookAtCommandExecutor(), this);

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
        getMovementController().addMovement(new JumpMovement());
    }

    public void lookAt(Vector3d target) {
        getMovementController().addMovement(new LookAtMovement(target));
    }

    public Vector3d getHeadPosition() {
        return new Vector3d(MovementSync.Instance.position.get())
                .add(Direction.UP.getVector(1.62));
    }
}
