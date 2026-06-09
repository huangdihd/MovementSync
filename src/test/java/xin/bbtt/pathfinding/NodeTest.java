package xin.bbtt.pathfinding;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class NodeTest {

    @Test
    void equalNodesHaveSameHashCode() {
        Node a = new Node(1, 2, 3);
        Node b = new Node(1, 2, 3);
        assertEquals(a, b);
        assertEquals(a.hashCode(), b.hashCode());
    }

    @Test
    void differentNodesAreNotEqual() {
        assertNotEquals(new Node(1, 2, 3), new Node(3, 2, 1));
    }

    @Test
    void toVectorRoundsTrips() {
        Node n = new Node(-5, 10, 42);
        assertEquals(n, new Node(n.toVector()));
    }
}
