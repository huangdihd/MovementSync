package xin.bbtt.pathfinding;

import static org.junit.jupiter.api.Assertions.*;

import java.util.*;

import org.junit.jupiter.api.Test;

class DStarLiteTest {

    @Test
    void straightLineOnEmptyPlane() {
        Node start = new Node(0, 0, 0);
        Node goal = new Node(5, 0, 0);
        GridContext ctx = new GridContext(Set.of());

        List<Node> path = new DStarLite(start, goal, ctx).findPath(200);
        assertFalse(path.isEmpty(), "path should be found on clear terrain");
        assertEquals(start, path.get(0));
        assertEquals(goal, path.get(path.size() - 1));
        assertTrue(path.size() >= 6, "path should be at least 6 nodes");
    }

    @Test
    void unreachableGoalReturnsEmpty() {
        Node start = new Node(0, 0, 0);
        Node goal = new Node(5, 0, 0);
        // Block all neighbours of start
        GridContext ctx = new GridContext(Set.of(
                new Node(1, 0, 0),
                new Node(-1, 0, 0),
                new Node(0, 0, 1),
                new Node(0, 0, -1)
        ));

        List<Node> path = new DStarLite(start, goal, ctx).findPath(200);
        assertTrue(path.isEmpty(), "no path when all exits are blocked from start");
    }

    @Test
    void detoursAroundObstacle() {
        Node start = new Node(0, 0, 0);
        Node goal = new Node(2, 0, 0);
        // Block the direct neighbour (1,0,0) so the path must go around.
        GridContext ctx = new GridContext(Set.of(new Node(1, 0, 0)));

        List<Node> path = new DStarLite(start, goal, ctx).findPath(200);
        assertFalse(path.isEmpty());
        assertEquals(start, path.get(0));
        assertEquals(goal, path.get(path.size() - 1));
    }

    @Test
    void iterationLimitReturnsPartial() {
        Node start = new Node(0, 0, 0);
        Node goal = new Node(100, 0, 0);
        GridContext ctx = new GridContext(Set.of());

        List<Node> path = new DStarLite(start, goal, ctx).findPath(10);
        // Should stop early due to iteration cap; may return partial.
        // The important assertion is it doesn't loop forever.
        assertNotNull(path);
        assertTrue(path.size() < 100, "should stop early — path shorter than direct line");
    }

    @Test
    void startEqualsGoalReturnsSingleNodePath() {
        Node same = new Node(3, 0, 7);
        GridContext ctx = new GridContext(Set.of());
        List<Node> path = new DStarLite(same, same, ctx).findPath(200);
        assertFalse(path.isEmpty());
        assertEquals(1, path.size());
        assertEquals(same, path.get(0));
    }
}
