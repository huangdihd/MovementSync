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
    private final xin.bbtt.physics.FlightControlAdmission flightControls = new xin.bbtt.physics.FlightControlAdmission();
    private long physicsTickVersion;

    public synchronized long physicsTickVersion() { return physicsTickVersion; }

    /** Advances the physical clock only after a physics step's result has been committed. */
    public synchronized void completePhysicsStep() {
        physicsTickVersion++;
        flightControls.completePhysicsStep();
        flightTickCursor = new FlightTickCursor(physicsTickVersion, flightTickContinuity);
    }

    /** Production boundary: publish the committed physical state with its clock atomically. */
    public synchronized void completePhysicsStep(FlightTickTrace trace) {
        physicsTickVersion++;
        flightControls.completePhysicsStep();
        recordFlightTick(trace);
        recentFlightTicks.addLast(new FlightTickSample(physicsTickVersion, flightTickContinuity, lastFlightTick));
        while (recentFlightTicks.size() > 64) recentFlightTicks.removeFirst();
        flightTickCursor = new FlightTickCursor(physicsTickVersion, flightTickContinuity);
    }

    public synchronized void submitFlightControls(xin.bbtt.physics.FlightControlAdmission.Envelope envelope) {
        flightControls.submit(envelope);
    }

    public synchronized boolean registerForwardFlightEmergency(Object owner,
            xin.bbtt.physics.FlightControlAdmission.Source source, float yaw) {
        return registerForwardFlightEmergency(owner, source, yaw, null);
    }

    public synchronized boolean registerForwardFlightEmergency(Object owner,
            xin.bbtt.physics.FlightControlAdmission.Source source, float yaw,
            xin.bbtt.physics.WaypointFlightTarget route) {
        return flightControls.registerForwardEmergency(this, owner, source, yaw, route);
    }

    public synchronized boolean refineForwardFlightEmergency(Object owner,
            xin.bbtt.physics.FlightControlAdmission.Source source, float yaw,
            xin.bbtt.physics.WaypointFlightTarget route) {
        return flightControls.refineForwardEmergency(this, owner, source, yaw, route);
    }

    public synchronized boolean publishFlightPrefix(Object owner,
            xin.bbtt.physics.FlightControlAdmission.Source source,
            java.util.List<xin.bbtt.physics.FlightControlAdmission.Step> steps,
            java.util.List<xin.bbtt.physics.FlightControlAdmission.Step> recovery,
            xin.bbtt.physics.WaypointFlightTarget route, Runnable accepted) {
        return publishFlightPrefix(owner, source, steps, recovery, route, accepted, false);
    }

    public synchronized boolean publishFlightPrefix(Object owner,
            xin.bbtt.physics.FlightControlAdmission.Source source,
            java.util.List<xin.bbtt.physics.FlightControlAdmission.Step> steps,
            java.util.List<xin.bbtt.physics.FlightControlAdmission.Step> recovery,
            xin.bbtt.physics.WaypointFlightTarget route, Runnable accepted,
            boolean requireAirborne) {
        return flightControls.publishPrefix(this, owner, source, steps, recovery, route, accepted,
                requireAirborne);
    }

    public synchronized void revokeFlightControls(Object owner) { flightControls.revoke(owner); }

    public synchronized void claimFlightControlOwner(Object owner, xin.bbtt.physics.FlightControlAdmission.Mode mode) {
        flightControls.claim(owner, mode);
    }

    public synchronized long flightControlEpoch() { return flightControls.epoch(); }

    /** Host-level handoff also retires batches owned by nested/plugin movements. */
    public synchronized void invalidateFlightControls() { flightControls.clear(); }

    public synchronized xin.bbtt.physics.FlightControlAdmission.Failure getFlightControlFailure() {
        return flightControls.failure();
    }

    /** Shared physical-input boundary, including embedded/manual physics drivers. */
    public synchronized PlayerInput snapshotInput() {
        flightControls.consume(this);
        return snapshotRawInput();
    }

    /** Revalidates current controls while preserving only the already-admitted deferred batch. */
    public synchronized PlayerInput snapshotDeferredDeploymentInput() {
        long deferredEpoch = flightControls.epoch();
        flightControls.consume(this);
        long currentEpoch = flightControls.epoch();
        if (currentEpoch != deferredEpoch)
            flightActionEpochs.replaceAll((packet, epoch) -> epoch == deferredEpoch ? currentEpoch : epoch);
        return snapshotRawInput();
    }

    private PlayerInput snapshotRawInput() {
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

    private final java.util.Map<Packet, Long> flightActionEpochs = new java.util.IdentityHashMap<>();
    private boolean acceptingFlightAction;

    /** Admission-only action scope; already-sent rockets remain real server state after handoff. */
    public synchronized void acceptFlightAction(Runnable action) {
        acceptingFlightAction = true;
        try { action.run(); } finally { acceptingFlightAction = false; }
    }

    public synchronized void queueBeforeMovement(Packet packet) {
        Objects.requireNonNull(packet, "packet");
        if (acceptingFlightAction) flightActionEpochs.put(packet, flightControls.epoch());
        beforeMovementPackets.add(packet);
    }

    public synchronized Packet pollBeforeMovementPacket() {
        Packet packet;
        while ((packet = beforeMovementPackets.poll()) != null) {
            Long epoch = flightActionEpochs.get(packet);
            if (epoch == null || epoch == flightControls.epoch()) return packet;
            flightActionEpochs.remove(packet);
            if (packet == pendingFireworkSlotSelection) pendingFireworkSlotSelection = null;
            if (packet == pendingFireworkRequest) pendingFireworkRequest = null;
        }
        return null;
    }

    public enum BeforeMovementDispatch { SENT, DISCARDED, RETRY }

    /** Sends a polled action, confirming firework cooldown only after session.send accepts it. */
    public synchronized BeforeMovementDispatch sendBeforeMovementPacket(Packet queuedPacket, Packet outboundPacket) {
        Objects.requireNonNull(queuedPacket, "queuedPacket");
        Objects.requireNonNull(outboundPacket, "outboundPacket");
        Long epoch = flightActionEpochs.get(queuedPacket);
        if (epoch != null && epoch != flightControls.epoch()) {
            flightActionEpochs.remove(queuedPacket);
            if (queuedPacket == pendingFireworkSlotSelection) pendingFireworkSlotSelection = null;
            if (queuedPacket == pendingFireworkRequest) pendingFireworkRequest = null;
            return BeforeMovementDispatch.DISCARDED;
        }
        var session = Bot.INSTANCE == null ? null : Bot.INSTANCE.getSession();
        if (session == null || !session.isConnected()) return BeforeMovementDispatch.RETRY;
        try {
            session.send(outboundPacket);
        } catch (RuntimeException error) {
            return BeforeMovementDispatch.RETRY;
        }
        flightActionEpochs.remove(queuedPacket);
        if (queuedPacket == pendingFireworkSlotSelection) pendingFireworkSlotSelection = null;
        if (queuedPacket == pendingFireworkRequest) {
            lastFireworkRequestTick = physicsTickVersion;
            pendingFireworkRequest = null;
        }
        if (outboundPacket instanceof org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundSetCarriedItemPacket carried)
            inventoryManager.setHeldSlot(carried.getSlot());
        return BeforeMovementDispatch.SENT;
    }

    public synchronized boolean hasQueuedGlideDeployment() {
        return beforeMovementPackets.stream().anyMatch(packet ->
                packet instanceof ServerboundPlayerCommandPacket command
                && command.getState() == PlayerState.START_ELYTRA_FLYING);
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
    public record FlightTickTrace(Vector3d position, Vector3d velocity, boolean onGround,
            boolean collidedX, boolean collidedZ, boolean gliding, boolean attached,
            float yaw, float pitch, PlayerInput input, double fallDistance) {
        public FlightTickTrace {
            position = new Vector3d(position);
            velocity = new Vector3d(velocity);
        }
        @Override public Vector3d position() { return new Vector3d(position); }
        @Override public Vector3d velocity() { return new Vector3d(velocity); }
    }
    public record FlightTickSample(long tick, long continuity, FlightTickTrace trace) {}
    public record FlightTickCursor(long tick, long continuity) {}
    private volatile FlightTickCursor flightTickCursor = new FlightTickCursor(0, 0);
    /** Initialization may hold navigation/queue locks: never acquire the physics monitor here. */
    public FlightTickCursor flightTickCursor() { return flightTickCursor; }
    private volatile FlightTickTrace lastFlightTick;
    private final java.util.ArrayDeque<FlightTickSample> recentFlightTicks = new java.util.ArrayDeque<>();
    private long flightTickContinuity;
    public FlightTickTrace getLastFlightTick() { return lastFlightTick; }
    public synchronized long flightTickContinuity() { return flightTickContinuity; }
    public synchronized java.util.List<FlightTickSample> flightTicksAfter(long tick, long continuity) {
        return recentFlightTicks.stream()
                .filter(sample -> sample.continuity() == continuity && sample.tick() > tick).toList();
    }
    public synchronized void invalidateFlightTickContinuity() {
        flightTickContinuity++;
        recentFlightTicks.clear();
        flightTickCursor = new FlightTickCursor(physicsTickVersion, flightTickContinuity);
    }
    public synchronized void recordFlightTick(FlightTickTrace trace) {
        lastFlightTick = trace;
        if (physicsTraceEnabled) {
            lastPhysicsTickTrace = xin.bbtt.flight.PhysicsTickTrace.copy(++physicsTraceSequence,
                    physicsTickVersion, getNavigationGeneration(), flightControls.ownershipTrace(), trace);
        }
    }

    private final xin.bbtt.flight.ScopeRouter flightScopeRouter = new xin.bbtt.flight.ScopeRouter();
    private boolean physicsTraceEnabled;
    private long physicsTraceSequence;
    private volatile xin.bbtt.flight.PhysicsTickTrace lastPhysicsTickTrace;

    /** P0 dispatch seam; all routes still execute the legacy controller. */
    public xin.bbtt.flight.ScopeRouter getFlightScopeRouter() { return flightScopeRouter; }

    /** Opt-in, latest-only diagnostics. Does not advance or repair the legacy input clock. */
    public synchronized void setPhysicsTraceEnabled(boolean enabled) {
        physicsTraceEnabled = enabled;
        if (!enabled) lastPhysicsTickTrace = null;
    }

    public xin.bbtt.flight.PhysicsTickTrace getLastPhysicsTickTrace() { return lastPhysicsTickTrace; }

    private final xin.bbtt.flight.ShadowFlightDiagnostics shadowFlightDiagnostics =
            new xin.bbtt.flight.ShadowFlightDiagnostics();
    public xin.bbtt.flight.ShadowFlightDiagnostics getShadowFlightDiagnostics() { return shadowFlightDiagnostics; }

    private final AtomicBoolean gliding = new AtomicBoolean();
    private boolean awaitingNewFlight;
    private volatile boolean dead;

    public boolean isDead() { return dead; }
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
        if (!java.util.Objects.equals(recoveryDimension, dimension)) invalidateFlightTickContinuity();
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
            clearQueuedSessionActions();
            reconnectAirRecovery.reset();
            if (gotoRecovery.isPending()) getLogger().info("Static goto suspended for reconnect");
            advanceNavigationGeneration();
            movementController.cancelAll();
            activeGoal = null;
            followTargetId = -1;
            clearInput();
        }
    }

    /** Called after the authoritative first position has been applied. */
    public void onRecoveryTeleport() {
        if (gotoRecovery.onTeleport()) reconnectAirRecovery.positioned();
        else if (!gotoRecovery.isPending()) {
            // A correction applies authoritative position/velocity, not a user stop.
            // Follow replans from that state; retain its owner and generation rather
            // than replacing it with an independent landing that forgets the entity.
            synchronized (navigationLock) {
                if (followTargetId != -1) return;
                cancelNavigation(false, "authoritative position correction");
            }
        }
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

    /** Metadata from the previous life must not restart flight after death/respawn. */
    public synchronized byte applySharedFlags(byte flags) {
        if (awaitingNewFlight || dead) flags = (byte) (flags & ~0x80);
        boolean flying = (flags & 0x80) != 0;
        if (flying && !isGliding()) getLogger().info("Server metadata: adopting authoritative gliding state");
        setGliding(flying);
        return flags;
    }

    /** A new connection can legitimately reconnect in mid-flight. */
    public synchronized void resetFlightMetadataSession() {
        dead = false;
        awaitingNewFlight = false;
        naturalDescentGeneration = Long.MIN_VALUE;
        invalidateFlightTickContinuity();
        clearFlightState();
    }

    public void onPlayerDeath() {
        resetRespawnFlightState(true);
        landingMemory.clear();
        cancelNavigation(false);
    }

    public void onPlayerRespawn() {
        resetRespawnFlightState(false);
    }

    private synchronized void clearQueuedSessionActions() {
        beforeMovementPackets.clear();
        flightActionEpochs.clear();
        pendingFireworkSlotSelection = null;
        pendingFireworkRequest = null;
    }

    private synchronized void resetRespawnFlightState(boolean dead) {
        this.dead = dead;
        awaitingNewFlight = true;
        invalidateFlightTickContinuity();
        clearFlightState();
        clearInput();
        clearQueuedSessionActions();
        velocity.set(new Vector3d());
        fallDistance.set(0.0);
        vehicleId = -1;
    }

    /** Returns whether a start request was sent, not whether the server accepted it. */
    public synchronized boolean startGliding() {
        if (dead || isGliding() || onGround.get() || isRiding() || entityId < 0) return false;
        if (!inventoryManager.canStartFlight(0)) return false;
        if (Bot.INSTANCE == null || Bot.INSTANCE.getSession() == null || !Bot.INSTANCE.getSession().isConnected()) return false;
        awaitingNewFlight = false;
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
    private long lastFireworkRequestTick = Long.MIN_VALUE;
    private Packet pendingFireworkSlotSelection;
    private ServerboundUseItemPacket pendingFireworkRequest;

    /** Shared request latch survives navigation handoffs; requests are not attachments. */
    public synchronized boolean fireworkRequestReady() {
        return pendingFireworkRequest == null
                && (lastFireworkRequestTick == Long.MIN_VALUE || physicsTickVersion - lastFireworkRequestTick >= 40);
    }

    public synchronized boolean useFirework() {
        if (!isGliding() || !fireworkRequestReady()) return false;
        int rocketSlot = hotbarFireworks().firstSlot();
        if (rocketSlot < 0) return false;
        if (Bot.INSTANCE == null || Bot.INSTANCE.getSession() == null || !Bot.INSTANCE.getSession().isConnected()) return false;
        if (inventoryManager.getHeldSlot() != rocketSlot) {
            pendingFireworkSlotSelection = new org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundSetCarriedItemPacket(rocketSlot);
            queueBeforeMovement(pendingFireworkSlotSelection);
        }
        pendingFireworkRequest = new ServerboundUseItemPacket(
                Hand.MAIN_HAND, Bot.INSTANCE.getAndIncreaseSequence(), yaw.get(), pitch.get());
        queueBeforeMovement(pendingFireworkRequest);
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

    private final ThreadLocal<Boolean> physicsTickAttached = new ThreadLocal<>();

    /** Freeze asynchronous entity metadata for one complete outbound physics/input batch. */
    public synchronized void beginPhysicsTickState() {
        physicsTickAttached.set(hasLiveAttachedFirework());
    }

    public synchronized void endPhysicsTickState() {
        physicsTickAttached.remove();
    }

    public boolean hasAttachedFirework() {
        Boolean attached = physicsTickAttached.get();
        return attached != null ? attached : hasLiveAttachedFirework();
    }

    private boolean hasLiveAttachedFirework() {
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
            long generation = advanceNavigationGeneration();
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
            long generation = advanceNavigationGeneration();
            movementController.cancelAll();
            return generation;
        }
    }

    /** Releases an unpublished request, retaining reconnect intent for a later recovery attempt. */
    public void abandonNavigationRequest(long generation) {
        synchronized (navigationLock) {
            if (!isNavigationRequestCurrent(generation)) return;
            activeGoal = null;
        }
    }

    private volatile long naturalDescentGeneration = Long.MIN_VALUE;

    /** A terminal failed request relinquishes even the ownerless hazard fallback. */
    public synchronized boolean requiresNaturalDescent() {
        return naturalDescentGeneration == navigationGeneration.get();
    }

    /** Failure is terminal, but never an arrival; stale owners cannot clear newer intent. */
    public void abandonNavigationRequest(long generation, String reason) {
        synchronized (navigationLock) {
            if (!isNavigationRequestCurrent(generation)) return;
            activeGoal = null;
            clearRecoverableGoto("failed: " + reason);
            advanceNavigationGenerationForFailure();
            movementController.cancelAll();
            safetyLanding = null;
            clearInput();
            clearAbandonedFlightActions(false);
            followTargetId = -1;
        }
    }

    /** Discard abandoned flight actions without dropping inventory transaction packets.
     * Only an explicit user cancellation may retain an accepted glide's pending
     * start for the independent safety-landing owner.
     */
    private synchronized void clearAbandonedFlightActions(boolean preserveAcceptedGlideStart) {
        beforeMovementPackets.removeIf(packet -> packet == pendingFireworkSlotSelection
                || packet instanceof ServerboundUseItemPacket
                || (!preserveAcceptedGlideStart || !isGliding())
                && packet instanceof ServerboundPlayerCommandPacket command
                && command.getState() == PlayerState.START_ELYTRA_FLYING);
        flightActionEpochs.clear();
        pendingFireworkSlotSelection = null;
        pendingFireworkRequest = null;
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
        cancelNavigation(land, land ? "explicit stop or navigation replacement" : "death or lifecycle reset");
    }

    private void cancelNavigation(boolean land, String reason) {
        long safetyGeneration;
        xin.bbtt.movements.ElytraFlyMovement landing = null;
        synchronized (navigationLock) {
            if (land && !dead && safetyLanding != null && !safetyLanding.isFinished()) {
                getLogger().info("正在安全落地,落地后停止");
                return;
            }
            clearRecoverableGoto(reason);
            if (land && !dead && !isRiding() && isGliding())
                landing = new xin.bbtt.movements.ElytraFlyMovement(position.get()).forIndependentLanding();
            safetyGeneration = landing == null
                    ? advanceNavigationGeneration()
                    : advanceNavigationGenerationForLanding(landing);
            activeGoal = null;
            followTargetId = -1;
            safetyLanding = landing;
            clearInput();
            clearAbandonedFlightActions(land);
            if (landing == null) movementController.cancelAll();
            else movementController.cancelAllPreservingFlightControls();
        }
        if (land) startSafetyExit(safetyGeneration, landing);
    }

    /** Terrain checks run outside the navigation lock; publication rechecks ownership. */
    private void startSafetyExit(long generation, xin.bbtt.movements.ElytraFlyMovement landing) {
        if (dead || isRiding()) return;
        boolean hazard = landing == null && !isGliding()
                && xin.bbtt.movements.HazardExitMovement.needed(this);
        synchronized (navigationLock) {
            if (!isNavigationRequestCurrent(generation) || dead || isRiding()) return;
            if (landing != null && safetyLanding == landing) {
                movementController.addMovement(landing);
            } else if (hazard && !isGliding()) {
                movementController.addMovement(new xin.bbtt.movements.HazardExitMovement(generation));
            }
        }
    }

    /** Serialize invalidation with physical snapshots/admission, without taking navigationLock
     * from inside the physics monitor. Callers already own navigationLock. */
    private synchronized long advanceNavigationGeneration() {
        flightControls.clear();
        return navigationGeneration.incrementAndGet();
    }

    private synchronized long advanceNavigationGenerationForLanding(
            xin.bbtt.movements.ElytraFlyMovement landing) {
        flightControls.clear();
        flightControls.retireFallback(this);
        long generation = navigationGeneration.incrementAndGet();
        flightControls.claim(landing, xin.bbtt.physics.FlightControlAdmission.Mode.DIRECT);
        return generation;
    }

    private synchronized long advanceNavigationGenerationForFailure() {
        flightControls.clear();
        flightControls.retireFallback(this);
        long generation = navigationGeneration.incrementAndGet();
        naturalDescentGeneration = generation;
        return generation;
    }

    // Lock-free by design: physical admission reads this while holding MovementSync.
    // Never acquire navigationLock here (navigation cancellation takes the opposite order).
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
