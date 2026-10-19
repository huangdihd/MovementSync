package xin.bbtt.tasks;

import static org.junit.jupiter.api.Assertions.*;
import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.*;
import org.joml.Vector3d;
import org.junit.jupiter.api.*;
import org.geysermc.mcprotocollib.network.ClientSession;
import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket;
import xin.bbtt.MovementSync;
import xin.bbtt.Entity.Entity;
import xin.bbtt.inventory.ItemRegistry;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.Server;
import xin.bbtt.world.World;
import xin.bbtt.physics.*;

class P5eFallbackOwnerTest {
    MovementSync sync;
    updateMotionTask motion;
    List<Object> packets = new ArrayList<>();
    boolean supported, ceiling, lava, unknown;

    @BeforeEach void setup() throws Exception {
        sync = new MovementSync();
        sync.onRecoveryTeleport();
        sync.entityId = 7;
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return !unknown; }
            @Override public xin.bbtt.Block.BlockState getBlockStateAt(Vector3d p) {
                if (lava && p.y < 7) return new xin.bbtt.Block.BlockState("minecraft:lava", 0,
                        Map.of(), "empty", 0, false, "LAVA");
                return super.getBlockStateAt(p);
            }
            @Override public int getBlockAt(Vector3d p) {
                return supported && p.y < 1 || ceiling && p.y >= 12 ? 1 : 0;
            }
        };
        Field field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true); field.set(sync, world);
        bot("running", true);
        Bot.INSTANCE.setServer(Server.Game);
        bot("session", Proxy.newProxyInstance(ClientSession.class.getClassLoader(), new Class<?>[]{ClientSession.class},
                (p, m, a) -> {
                    if (m.getName().equals("isConnected")) return true;
                    if (m.getName().equals("send")) packets.add(a[0]);
                    if (m.getReturnType() == boolean.class) return false;
                    if (m.getReturnType() == int.class) return 0;
                    return null;
                }));
        sync.position.set(new Vector3d(.5, 10, .5));
        sync.velocity.set(new Vector3d(0, -.08, 0));
        sync.onGround.set(false); sync.setGliding(true);
        sync.yaw.set(0f); sync.pitch.set(0f);
        sync.inventoryManager.setContainerItems(0, new ItemStack[46]);
        sync.inventoryManager.getInventory()[36] = new ItemStack(ItemRegistry.Instance.getItem("firework_rocket").getId(), 8);
        motion = new updateMotionTask();
    }
    @AfterEach void cleanup() throws Exception { sync.clearFlightState(); bot("session", null); bot("running", false); }
    static void bot(String name, Object value) throws Exception {
        Field f = Bot.class.getDeclaredField(name); f.setAccessible(true); f.set(Bot.INSTANCE, value);
    }
    long uses() { return packets.stream().filter(ServerboundUseItemPacket.class::isInstance).count(); }
    void attach() {
        Entity rocket = new Entity(42, UUID.randomUUID(), EntityType.FIREWORK_ROCKET,
                .5, 10, .5, 0, 0, 0, new Vector3d());
        rocket.getMetadata().put("attached_to_target", OptionalInt.of(sync.entityId));
        sync.getWorld().getEntities().put(42, rocket);
    }
    @Test void routeAbsentRequestsAndConfirmedAttachmentClimbs() {
        lava = true;
        motion.run();
        assertEquals(1, uses(), "unowned exposed descent must receive a checked request on first tick");
        assertFalse(sync.hasAttachedFirework(), "request must not manufacture attachment");
        assertTrue(sync.velocity.get().y < 0, "unconfirmed request must not manufacture thrust");
        attach();
        for (int i = 0; i < 8; i++) motion.run();
        assertTrue(sync.velocity.get().y > 0);
        assertTrue(sync.position.get().y > 10);
        assertEquals(1, uses());
    }
    @Test void invalidationCannotCreateAnUnownedHazardWindowOrResetCooldown() {
        lava = true;
        sync.claimFlightControlOwner(this, FlightControlAdmission.Mode.LOCAL_SOLVER);
        sync.invalidateFlightControls();
        for (int i = 0; i < 12; i++) { sync.invalidateFlightControls(); motion.run(); }
        assertEquals(1, uses());
        assertTrue(sync.pitch.get() < 0);
    }
    @Test void noRocketsStillGetsCheckedAttitudeButNoInventedClimb() {
        lava = true;
        sync.inventoryManager.getInventory()[36] = null;
        motion.run();
        assertTrue(sync.pitch.get() < 0);
        assertEquals(0, uses());
        assertTrue(sync.velocity.get().y < 0);
    }
    @Test void exposedLavaRequestsBeforeContact() { lava = true; motion.run(); assertEquals(1, uses()); }
    @Test void unknownTerrainCannotAuthorizeBoost() { lava = true; unknown = true; motion.run(); assertEquals(0, uses()); }
    @Test void priorRequestCooldownSurvivesOwnerHandoff() {
        lava = true;
        assertTrue(sync.useFirework());
        sync.invalidateFlightControls(); motion.run();
        assertEquals(1, uses());
    }
    @Test void metadataStopDoesNotGetOverridden() {
        sync.setGliding(false); motion.run(); assertEquals(0, uses()); assertFalse(sync.isGliding());
    }
    @Test void poweredCeilingIsRejected() { lava = true; ceiling = true; motion.run(); assertEquals(0, uses()); }
    @Test void supportedFlightUnchanged() { supported = true; motion.run(); assertEquals(0f, sync.pitch.get()); assertEquals(0, uses()); }
    @Test void expiredOwnedPrefixFallsBackInTheSameMotionTick() {
        lava = true;
        sync.claimFlightControlOwner(this, FlightControlAdmission.Mode.DIRECT);
        var source = FlightControlAdmission.snapshot(sync);
        var control = new ElytraFlightController.Decision(0, -5, false, ElytraFlightController.Phase.CRUISE);
        sync.submitFlightControls(new FlightControlAdmission.Envelope(this, source,
                List.of(new FlightControlAdmission.Step(control, source.state())), null, () -> {}, List.of()));
        motion.run(); assertEquals(0, uses());
        motion.run(); assertEquals(1, uses()); assertEquals(-55f, sync.pitch.get());
    }
    @Test void highEmptyAirRetainsExternalControlWithoutInventingHazardOwner() {
        sync.pitch.set(23f);
        motion.run();
        assertEquals(23f, sync.pitch.get());
        assertEquals(0, uses());
        assertNull(sync.getFlightControlFailure());
    }
    @Test void loadedVoidNearWorldBottomStillGetsFallback() {
        sync.position.set(new Vector3d(.5, World.getMinWorldY() + 10, .5));
        motion.run();
        assertEquals(-55f, sync.pitch.get());
        assertEquals(1, uses());
    }
    @Test void healthyUnownedVoidClimbIsNotOverridden() {
        sync.velocity.set(new Vector3d(0, .2, 1.5));
        motion.run(); assertEquals(0, uses()); assertEquals(0f, sync.pitch.get());
    }
    @Test void admittedOwnerIsNotOverridden() {
        lava = true;
        sync.claimFlightControlOwner(this, FlightControlAdmission.Mode.DIRECT);
        var source = FlightControlAdmission.snapshot(sync);
        var control = new ElytraFlightController.Decision(0, -5, false, ElytraFlightController.Phase.CRUISE);
        sync.submitFlightControls(new FlightControlAdmission.Envelope(this, source,
                List.of(new FlightControlAdmission.Step(control, source.state())), null, () -> {}, List.of()));
        motion.run();
        assertEquals(-5f, sync.pitch.get()); assertEquals(0, uses());
    }
}
