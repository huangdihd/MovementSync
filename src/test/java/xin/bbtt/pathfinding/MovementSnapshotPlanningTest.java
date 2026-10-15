package xin.bbtt.pathfinding;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.physics.MovementParameters;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class MovementSnapshotPlanningTest {
    @Test
    void jumpBoostTwoReachesTwoBlockPlatformButDefaultDoesNot() {
        World world = platform(1, 2);
        Node source = new Node(0, 0, 0);
        Node target = new Node(1, 2, 0);
        try (var ignored = StandablePositionResolver.beginSearch(MovementParameters.DEFAULT)) {
            assertFalse(StandablePositionResolver.canJump(world, source, target));
        }
        try (var ignored = StandablePositionResolver.beginSearch(parameters(0.1, 0.2))) {
            assertTrue(StandablePositionResolver.canJump(world, source, target));
            assertTrue(new JumpStrategy().findEdges(source, world).stream()
                    .anyMatch(edge -> edge.getTarget().equals(target)));
        }
    }

    @Test
    void slownessRejectsGapThroughTheSamePhysicsReplay() {
        World world = platform(3, 0);
        Node source = new Node(0, 0, 0);
        Node target = new Node(3, 0, 0);
        try (var ignored = StandablePositionResolver.beginSearch(MovementParameters.DEFAULT)) {
            assertTrue(StandablePositionResolver.canGapJump(world, source, target));
        }
        try (var ignored = StandablePositionResolver.beginSearch(parameters(0.01, 0))) {
            assertFalse(StandablePositionResolver.canGapJump(world, source, target));
            assertFalse(new GapJumpStrategy().findEdges(source, world).stream()
                    .anyMatch(edge -> edge.getTarget().equals(target)));
        }
    }

    @Test
    void nestedSearchRestoresImmutableSnapshot() {
        MovementParameters boosted = parameters(0.1, 0.2);
        try (var outer = StandablePositionResolver.beginSearch(boosted)) {
            assertSame(boosted, StandablePositionResolver.movementParameters());
            try (var inner = StandablePositionResolver.beginSearch(MovementParameters.DEFAULT)) {
                assertSame(MovementParameters.DEFAULT, StandablePositionResolver.movementParameters());
            }
            assertSame(boosted, StandablePositionResolver.movementParameters());
        }
    }

    private static MovementParameters parameters(double speed, double boost) {
        return new MovementParameters(speed, speed * 1.3, (double) 0.42f, boost,
                0.08, 0.6, 0.3, false, -1, true);
    }

    private static World platform(int targetX, int height) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (z != 0) return 0;
                return (x == 0 && y == -1) || (x == targetX && y < height && y >= -1) ? 1 : 0;
            }
        };
    }
}
