package xin.bbtt.pathfinding;

import org.joml.Vector3i;
import org.joml.Vector3ic;

import java.util.Objects;

public class Node {
    public final int x, y, z;

    public Node(int x, int y, int z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Node(Vector3ic vec) {
        this.x = vec.x();
        this.y = vec.y();
        this.z = vec.z();
    }

    public Vector3i toVector() {
        return new Vector3i(x, y, z);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Node node = (Node) o;
        return x == node.x && y == node.y && z == node.z;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y, z);
    }

    @Override
    public String toString() {
        return String.format("(%d, %d, %d)", x, y, z);
    }
}
