package xin.bbtt.collision;

/** One block-local axis-aligned collision box. */
public record CollisionBox(
        double minX, double minY, double minZ,
        double maxX, double maxY, double maxZ) {

    public boolean contains(double x, double y, double z) {
        return x >= minX && x < maxX
            && y >= minY && y < maxY
            && z >= minZ && z < maxZ;
    }

    public boolean intersects(
            double otherMinX, double otherMinY, double otherMinZ,
            double otherMaxX, double otherMaxY, double otherMaxZ) {
        return minX < otherMaxX && maxX > otherMinX
            && minY < otherMaxY && maxY > otherMinY
            && minZ < otherMaxZ && maxZ > otherMinZ;
    }
}
