package xin.bbtt.pathfinding;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Set;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.physics.JumpInputController;
import xin.bbtt.physics.PhysicsResult;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.WalkingPhysics;
import xin.bbtt.world.World;

final class SharedJumpTrajectoryTest {
    private static final double PLAYER_HALF_WIDTH = (double) 0.6f / 2;
    @Test
    void previewMatchesKeyDrivenPhysicsForCardinalAndDiagonalGaps() {
        for (Node target : new Node[] {new Node(2, 1, 0), new Node(3, 1, 0),
                new Node(2, 1, 2), new Node(4, 1, 0)}) {
            World world = supports(target, false);
            boolean landed = replay(world, target);
            assertEquals(landed, StandablePositionResolver.canGapJump(
                    world, new Node(0, 1, 0), target), target.toString());
            if (target.x <= 3) assertTrue(landed, target.toString());
            else assertFalse(landed, "a four-block center distance is not reachable from rest");
        }
    }

    @Test
    void previewRejectsCeilingThatCutsOffTheGapArc() {
        Node target = new Node(3, 1, 0);
        World world = supports(target, true);
        assertFalse(replay(world, target));
        assertFalse(StandablePositionResolver.canGapJump(world, new Node(0, 1, 0), target));
    }

    @Test
    void collisionProjectionExcludesOnlyDigCellsForEveryCollisionQuery() {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) {
                return position.y >= 0 && position.y < 2 ? 1 : 0;
            }
        };
        Vector3i excluded = new Vector3i(0, 1, 0);
        World view = new ExcludedBlockCollisionView(world, Set.of(excluded));
        excluded.set(9, 9, 9);
        assertFalse(view.isBoxColliding(0.2, 1, 0.2, 0.8, 2, 0.8));
        assertTrue(view.isBoxColliding(1.2, 1, 0.2, 1.8, 2, 0.8));
        assertEquals(1, view.findHighestCollisionTop(0.2, 0, 0.2, 0.8, 2, 0.8).orElseThrow());
        assertTrue(view.isOnGround(new Vector3d(0.5, 1, 0.5)));
        assertFalse(view.isOnGround(new Vector3d(0.5, 2, 0.5)));
        assertTrue(world.isBoxColliding(0.2, 1, 0.2, 0.8, 2, 0.8), "preview must not mutate the world");
        assertTrue(view.findHighestCollisionTopExcluding(0.2, 0, 0.2, 0.8, 2, 0.8,
                Set.of(new Vector3i(0, 0, 0))).isEmpty());
    }

    @Test
    void controllerUsesKeysAndBrakesMomentumWithoutMutatingArguments() {
        Vector3d position = new Vector3d(0.5, 1, 0.5);
        Vector3d velocity = new Vector3d(0.2, 0, 0);
        var braking = JumpInputController.toward(position, position, velocity, true, false);
        assertTrue(braking.input().forward());
        assertEquals(90, braking.yaw(), 0.001);
        assertFalse(braking.input().jump());
        assertEquals(new Vector3d(0.5, 1, 0.5), position);
        assertEquals(new Vector3d(0.2, 0, 0), velocity);
        Vector3d target = new Vector3d(3.5, 1, 0.5);
        var launch = JumpInputController.toward(position, target, velocity, true, true);
        assertTrue(launch.input().jump());
        assertTrue(launch.input().sprint());
        assertFalse(JumpInputController.toward(position, target, velocity, false, true).input().jump());
    }

    private static boolean replay(World world, Node target) {
        PhysicsState state = new PhysicsState(new Vector3d(0.5, 1, 0.5), new Vector3d(), true);
        Vector3d destination = new Vector3d(target.x + 0.5, 1, target.z + 0.5);
        for (int tick = 0; tick < 40; tick++) {
            var command = JumpInputController.toward(
                    state.position(), destination, state.velocity(), state.onGround(), true);
            PhysicsResult result = new WalkingPhysics(command.input(), command.yaw())
                    .step(world, state, PLAYER_HALF_WIDTH, 1.8);
            Vector3d position = result.position();
            if (result.onGround() && Math.abs(position.y - 1) < 1e-7
                    && position.x + PLAYER_HALF_WIDTH > target.x && position.x - PLAYER_HALF_WIDTH < target.x + 1
                    && position.z + PLAYER_HALF_WIDTH > target.z && position.z - PLAYER_HALF_WIDTH < target.z + 1) return true;
            if (position.y < 0) return false;
            state = new PhysicsState(position, result.velocity(), result.onGround());
        }
        return false;
    }

    private static World supports(Node target, boolean ceiling) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (ceiling && y == 3) return 1;
                return y == 0 && ((x == 0 && z == 0) || (x == target.x && z == target.z)) ? 1 : 0;
            }
        };
    }
}
