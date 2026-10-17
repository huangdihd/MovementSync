package xin.bbtt.movements;

import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.pathfinding.*;
import xin.bbtt.world.World;
import java.lang.reflect.Field;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Set;
import static org.junit.jupiter.api.Assertions.*;

class AirborneTravelTest extends ElytraCommandTestSupport {
    private void position(double y) throws Exception {
        Field world = movementSync.getClass().getDeclaredField("world");
        world.setAccessible(true);
        world.set(movementSync, new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public OptionalDouble findHighestCollisionTopExcluding(double x, double y, double z,
                    double xx, double yy, double zz, Set<Vector3i> excluded) {
                return y <= 0 && yy >= 0 ? OptionalDouble.of(0) : OptionalDouble.empty();
            }
        });
        movementSync.position.set(new Vector3d(.5, y, .5));
        movementSync.velocity.set(new Vector3d(0, -.2, 0));
        movementSync.onGround.set(false);
    }

    private Object child(TravelMovement travel) throws Exception {
        Field field = TravelMovement.class.getDeclaredField("child");
        field.setAccessible(true);
        return field.get(travel);
    }

    @Test void dangerousFallSelectsFlightForCruiseAndLanding() throws Exception {
        for (boolean fly : new boolean[]{true, false}) {
            position(100);
            inventory[36] = item("firework_rocket", 64);
            movementSync.inventoryManager.setContainerItems(0, inventory);
            Vector3i goal = new Vector3i(fly ? 600 : 40, 0, 0);
            TravelMovement travel = new TravelMovement(goal, movementSync.beginStaticNavigationRequest(goal), fly);
            travel.onTick();
            assertInstanceOf(ElytraFlyMovement.class, child(travel));
            assertFalse(movementSync.isGliding(), "parent must not deploy directly");
            travel.onTick();
            assertTrue(movementSync.isGliding());
            movementSync.setGliding(false);
        }
    }

    @Test void walkingChildYieldsToParentAndDeploys() throws Exception {
        position(100);
        Vector3i goal = new Vector3i(40, 100, 0);
        long generation = movementSync.beginStaticNavigationRequest(goal);
        TravelMovement travel = new TravelMovement(goal, generation, false);
        PathMovement path = new PathMovement(List.of(
                new PathStep(new Node(0, 100, 0), BuiltinMovementType.WALK),
                new PathStep(new Node(1, 100, 0), BuiltinMovementType.WALK)),
                -1, false, goal, -1, generation).yieldingUnexpectedFalls();
        Field childField = TravelMovement.class.getDeclaredField("child");
        childField.setAccessible(true);
        childField.set(travel, path);
        movementSync.position.set(new Vector3d(.5, 99, 5));
        travel.onTick();
        assertEquals(PathMovement.Completion.UNEXPECTED_FALL, path.completion());
        assertInstanceOf(ElytraFlyMovement.class, child(travel));
        travel.onTick();
        assertTrue(movementSync.isGliding());
    }

    @Test void lowFallAndMissingElytraWait() throws Exception {
        for (boolean equipped : new boolean[]{true, false}) {
            position(equipped ? 2 : 100);
            inventory[6] = equipped ? item("elytra", 1) : null;
            movementSync.inventoryManager.setContainerItems(0, inventory);
            Vector3i goal = new Vector3i(40, 0, 0);
            TravelMovement travel = new TravelMovement(goal, movementSync.beginStaticNavigationRequest(goal), false);
            travel.onTick();
            assertNull(child(travel));
            assertFalse(movementSync.isGliding());
        }
    }

    @Test void edgeCorridorExcludesPlannedJumpAndDropButYieldsKnockback() throws Exception {
        position(99);
        for (BuiltinMovementType type : new BuiltinMovementType[]{BuiltinMovementType.JUMP, BuiltinMovementType.WALK}) {
            Vector3i goal = new Vector3i(1, 97, 0);
            long generation = movementSync.beginStaticNavigationRequest(goal);
            PathMovement path = new PathMovement(List.of(
                    new PathStep(new Node(0, 100, 0), BuiltinMovementType.WALK),
                    new PathStep(new Node(1, 97, 0), type)), -1, false, goal, -1, generation).yieldingUnexpectedFalls();
            Field index = PathMovement.class.getDeclaredField("currentIndex");
            index.setAccessible(true);
            index.set(path, 1);
            path.onTick();
            assertEquals(PathMovement.Completion.NORMAL, path.completion());
            movementSync.position.set(new Vector3d(.5, 99, 5));
            path.onTick();
            assertEquals(PathMovement.Completion.UNEXPECTED_FALL, path.completion());
            assertTrue(path.isFinished());
            movementSync.position.set(new Vector3d(.5, 99, .5));
        }
    }

    @Test void standalonePathKeepsWalkingAfterKnockback() throws Exception {
        position(99);
        Vector3i goal = new Vector3i(1, 97, 0);
        long generation = movementSync.beginStaticNavigationRequest(goal);
        // No travel parent (plain walking goto or follow): nobody would handle the hand-back.
        PathMovement path = new PathMovement(List.of(
                new PathStep(new Node(0, 100, 0), BuiltinMovementType.WALK),
                new PathStep(new Node(1, 97, 0), BuiltinMovementType.WALK)), -1, false, goal, -1, generation);
        Field index = PathMovement.class.getDeclaredField("currentIndex");
        index.setAccessible(true);
        index.set(path, 1);
        movementSync.position.set(new Vector3d(.5, 99, 5));
        path.onTick();
        assertEquals(PathMovement.Completion.NORMAL, path.completion());
        assertFalse(path.isFinished());
    }
}
