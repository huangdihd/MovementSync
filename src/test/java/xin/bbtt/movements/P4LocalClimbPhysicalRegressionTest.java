package xin.bbtt.movements;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.OptionalInt;
import java.util.UUID;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PlayerState;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundPlayerCommandPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket;
import org.cloudburstmc.nbt.NbtMap;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.Entity.Entity;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.flight.LocalClimbHandoff;
import xin.bbtt.listeners.RegistryDataListener;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

/** Synthetic loaded platforms, NOT captured server terrain or P5 follow control.
 * All post-setup positions come from shared walking/gliding physics. Only server
 * deployment metadata and rocket attachments are supplied by this fixture.
 */
class P4LocalClimbPhysicalRegressionTest extends ElytraCommandTestSupport {
    private static final int ROCKET_ID = 9401;
    private static final BlockState STONE = new BlockState("stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState("air", 0, Map.of(), "empty", 0, false, "AIR");

    private List<NbtMap> dimensions;
    private List<NbtMap> savedDimensions;
    private Field dimensionIndex;
    private int savedIndex;

    @BeforeEach @SuppressWarnings("unchecked")
    void saveDimensionRegistry() throws Exception {
        Field types = RegistryDataListener.class.getDeclaredField("dimensionTypes");
        types.setAccessible(true);
        dimensions = (List<NbtMap>) types.get(null);
        savedDimensions = new ArrayList<>(dimensions);
        dimensionIndex = RegistryDataListener.class.getDeclaredField("currentDimensionIndex");
        dimensionIndex.setAccessible(true);
        savedIndex = dimensionIndex.getInt(null);
    }

    @AfterEach
    void restoreDimensionRegistry() throws Exception {
        dimensions.clear();
        dimensions.addAll(savedDimensions);
        dimensionIndex.setInt(null, savedIndex);
    }

    @Test void overworldStaticGotoAtRecordedFollowStartStaysClearForFortyDeploymentTicks() throws Exception {
        fly(new Vector3d(245.5, 33, 20.5), "minecraft:overworld", Double.POSITIVE_INFINITY, false, false);
    }

    @Test void netherStaticGotoAtMinus3Point5Y36StaysClearForFortyDeploymentTicks() throws Exception {
        fly(new Vector3d(-3.5, 36, -1.5), "minecraft:the_nether", 128, false, false);
    }

    @Test void localClimbDoesNotHitSyntheticCeiling() throws Exception {
        fly(new Vector3d(-3.5, 36, -1.5), "minecraft:overworld", 60, false, false);
    }

    @Test void fiftyMillisecondWorkerDelayDoesNotCausePostDeploymentContact() throws Exception {
        fly(new Vector3d(245.5, 33, 20.5), "minecraft:overworld", Double.POSITIVE_INFINITY, true, false);
    }

    @Test void deniedDeploymentReturnsToSupportedPlatformWithoutRocket() throws Exception {
        fly(new Vector3d(245.5, 33, 20.5), "minecraft:overworld", Double.POSITIVE_INFINITY, false, true);
    }

    private void fly(Vector3d start, String dimension, double roof, boolean delayed, boolean denied) throws Exception {
        boolean nether = dimension.equals("minecraft:the_nether");
        boolean expectHandoff = !nether && !denied && Double.isInfinite(roof);
        dimensions.clear();
        dimensions.add(NbtMap.builder().putBoolean("has_ceiling", nether)
                .putInt("min_y", nether ? 0 : -64).putInt("height", nether ? 256 : 384)
                .putInt("logical_height", nether ? 128 : 384).build());
        dimensionIndex.setInt(null, 0);
        assertEquals(nether, RegistryDataListener.hasCeiling());
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                // P3 synthetic ramp: a supported Y=60 shelf, not an airborne goal.
                double floor = nether ? 36 + Math.max(0, Math.min(24, Math.floor((p.x - 120) / 2))) : start.y;
                return p.y < floor || p.y >= roof ? STONE : AIR;
            }
        };
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
        movementSync.movementController.pause();
        movementSync.setRecoveryDimension(dimension);
        movementSync.position.set(new Vector3d(start));
        movementSync.velocity.set(new Vector3d(0, -.0784, 0));
        movementSync.onGround.set(true);
        movementSync.yaw.set(-95.4657f);
        inventory[36] = item("firework_rocket", 64);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        // Deliberately a fixed distant goal: follow steering belongs to P5.
        var owner = new ElytraFlyMovement(nether ? new Vector3d(175.5, 60, -31.5)
                : new Vector3d(start).add(2000, 0, -190));
        if (nether) {
            assertSame(STONE, world.getBlockStateAt(new Vector3d(start.x, 128, start.z)));
            assertTrue(world.isOnGround(owner.getTarget()), "synthetic goal must have support");
            var plan = LowAltitudeFlightPlan.submit(world,
                    new PhysicsState(start, movementSync.velocity.get(), true, 0), owner.getTarget(),
                    movementSync.getMovementParameters(), false,
                    new HazardRecovery.Context(movementSync.inventoryManager.getHotbarFireworkFlightDurations(),
                            0, false, movementSync.getLandingMemory()));
            var result = plan.get(30, TimeUnit.SECONDS);
            assertNotNull(result, "real underroof deployment plan required: " + plan.status());
            owner.withLowAltitudePlan(result);
        }
        PlayerPose pose = PlayerPose.STANDING;
        int deployedAt = -1, deployments = 0, rocketTicks = 0, rockets = 0, lastRocket = -40;
        int postDeploymentTicks = 0;
        boolean jumped = false, delayedOnce = false, climbObserved = false;
        int handoffAt = -1, postHandoffTicks = 0;
        Future<?> delayTask = null;
        CountDownLatch releaseDelay = new CountDownLatch(1);
        try {
            owner.init();
            for (int tick = 0; tick < 120; tick++) {
                Object controller = field(owner, "localGoto");
                LocalClimbHandoff handoff = (LocalClimbHandoff) field(owner, "climbHandoff");
                boolean delayThisTick = delayed && !delayedOnce && movementSync.isGliding();
                if (delayThisTick) {
                    // Block the actual single worker before requesting the next solution.
                    // Keep it blocked through physics, then impose a real 50 ms delay.
                    CountDownLatch started = new CountDownLatch(1);
                    delayTask = ((ExecutorService) field(controller, "worker")).submit(() -> {
                        started.countDown();
                        try {
                            releaseDelay.await();
                            Thread.sleep(50);
                        } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
                    });
                    assertTrue(started.await(5, TimeUnit.SECONDS), "actual worker must start blocker");
                    delayedOnce = true;
                }
                owner.onTick();
                if (handoff.phase() == LocalClimbHandoff.Phase.LOCAL_CLIMB) climbObserved = true;
                if (handoff.phase() == LocalClimbHandoff.Phase.CRUISE && handoffAt < 0) {
                    assertTrue(climbObserved, "cruise must follow an observed local climb");
                    handoffAt = tick;
                }
                if (handoffAt >= 0) {
                    assertEquals(LocalClimbHandoff.Phase.CRUISE, handoff.phase(), "handoff is irreversible");
                    postHandoffTicks++;
                }
                if (delayThisTick) {
                    assertFalse(delayTask.isDone(), "worker must remain delayed during control tick");
                    assertTrue((boolean) field(controller, "busy"), "real solve must be pending behind blocker");
                }
                // Input admission can enqueue the rocket: drain packets in the same physical tick.
                var input = movementSync.snapshotInput();
                Object packet;
                while ((packet = movementSync.pollBeforeMovementPacket()) != null) {
                    if (packet instanceof ServerboundPlayerCommandPacket command
                            && command.getState() == PlayerState.START_ELYTRA_FLYING) {
                        assertFalse(movementSync.onGround.get(), "must deploy after a physical jump");
                        assertTrue(movementSync.velocity.get().y <= 0, "deploy on descent");
                        deployments++;
                        if (deployedAt < 0) deployedAt = tick;
                        movementSync.applySharedFlags((byte) (denied ? 0 : 0x80));
                    } else if (packet instanceof ServerboundUseItemPacket) {
                        assertFalse(denied, "denied deployment must not request a rocket");
                        assertTrue(movementSync.isGliding());
                        assertEquals(0, rocketTicks, "no duplicate rocket while attached; handoffAt=" + handoffAt);
                        assertTrue(tick - lastRocket >= 40, "climb/cruise must share rocket cooldown; handoffAt=" + handoffAt + " tick=" + tick + " lastRocket=" + lastRocket + " physical=" + movementSync.physicsTickVersion());
                        lastRocket = tick;
                        rockets++;
                        inventory[36] = item("firework_rocket", 64 - rockets);
                        movementSync.inventoryManager.setContainerItems(0, inventory);
                        rocketTicks = 30;
                        var rocket = new Entity(ROCKET_ID, new UUID(0, ROCKET_ID), EntityType.FIREWORK_ROCKET,
                                movementSync.position.get().x, movementSync.position.get().y,
                                movementSync.position.get().z, 0, 0, 0, new Vector3d());
                        rocket.getMetadata().put("attached_to_target", OptionalInt.of(movementSync.entityId));
                        world.getEntities().put(ROCKET_ID, rocket);
                    }
                }
                boolean gliding = movementSync.isGliding();
                var state = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(),
                        movementSync.onGround.get(), movementSync.fallDistance.get(), pose);
                double height = (double) (float) pose.height();
                var next = gliding
                        ? new GlidingPhysics(movementSync.yaw.get(), movementSync.pitch.get(),
                                movementSync.hasAttachedFirework(), input, movementSync.getMovementParameters())
                                .step(world, state, PlayerPose.HALF_WIDTH, height)
                        : new WalkingPhysics(input, movementSync.yaw.get(), true, movementSync.getMovementParameters())
                                .step(world, state, PlayerPose.HALF_WIDTH, height);
                String trace = "tick=" + tick + " deployedAt=" + deployedAt + " p=" + next.position();
                assertTrue(next.position().isFinite(), trace);
                assertFalse(next.collidedX() || next.collidedZ(), trace);
                assertTrue(next.position().y >= start.y, "floor penetration: " + trace);
                assertTrue(next.position().y + height <= roof + 1e-6, "roof penetration: " + trace);
                if (next.position().y > start.y + .2) jumped = true;
                if (!denied && deployedAt >= 0) {
                    postDeploymentTicks++;
                    assertTrue(next.position().y + height < roof - 1e-6, "ceiling contact: " + trace);
                    assertFalse(next.onGround(), "post-deployment contact: " + trace);
                    assertTrue(next.position().y > start.y, trace);
                    assertTrue(movementSync.isGliding(), trace);
                    assertFalse(owner.isFinished(), "must not end a distant leg: " + trace);
                }
                movementSync.position.set(next.position());
                movementSync.velocity.set(next.velocity());
                movementSync.onGround.set(next.onGround());
                movementSync.fallDistance.set(next.fallDistance());
                movementSync.completePhysicsStep();
                pose = pose.afterTravel(world, next.position(), gliding, false, input.sneak());
                if (rocketTicks > 0 && --rocketTicks == 0) world.getEntities().remove(ROCKET_ID);
                if (delayThisTick) {
                    assertFalse(delayTask.isDone(), "physical safety must hold while solver is blocked");
                    releaseDelay.countDown();
                    delayTask.get(5, TimeUnit.SECONDS);
                }
                // Advance physics BEFORE waiting: one physical tick sees the outstanding
                // delayed request. Subsequent decisions use only genuine worker results.
                var await = controller.getClass().getDeclaredMethod("awaitIdle", long.class, TimeUnit.class);
                await.setAccessible(true);
                await.invoke(controller, 30L, TimeUnit.SECONDS);
                if (!denied && postDeploymentTicks >= 40
                        && (!expectHandoff || postHandoffTicks >= 41)) break;
            }
            assertTrue(jumped, "fixture must exercise walking jump physics");
            assertTrue(deployments > 0, "vacuous non-deployment is not a passing flight");
            if (denied) {
                assertTrue(movementSync.onGround.get(), "denial must return to support");
                assertFalse(movementSync.isGliding());
                assertEquals(0, rockets);
            } else {
                if (expectHandoff) {
                    assertTrue(postDeploymentTicks >= 40, "must observe at least forty physical ticks after deployment");
                } else {
                    assertEquals(40, postDeploymentTicks, "must observe forty physical ticks after deployment");
                }
                if (!nether) assertTrue(climbObserved, "must exercise actual local-climb phase");
                if (nether) assertTrue((boolean) field(owner, "lowAltitude"), "Nether must use underroof goto");
                if (expectHandoff) {
                    assertTrue(handoffAt >= 0, "must observe actual cruise handoff");
                    assertTrue(postHandoffTicks >= 41,
                            "attachment and cooldown checks must continue a full cooldown AFTER handoff");
                }
                assertEquals(1, deployments, "no deploy/land/redeploy cycle");
                assertTrue(rockets > 0, "exercise actual rocket admission");
                if (delayed) assertTrue(delayedOnce);
            }
        } finally {
            releaseDelay.countDown();
            if (delayTask != null) delayTask.cancel(true);
            owner.onStop();
            world.getEntities().remove(ROCKET_ID);
        }
    }

    private static Object field(Object object, String name) throws Exception {
        Field field = object.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return field.get(object);
    }
}
