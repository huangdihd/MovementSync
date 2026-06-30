package xin.bbtt.pathfinding;

import xin.bbtt.movement.Movement;

/**
 * The concrete movement an edge represents — how the bot travels from the
 * edge's source node to its target node.
 * <p>
 * Other plugins can implement this interface to define their own movement
 * types and attach them to the edges produced by their own
 * {@link MovementStrategy}. See {@link BuiltinMovementType} for the
 * standard types.
 */
public interface MovementType {
    String name();

    /**
     * Creates the Movement that traverses this edge, or {@code null} to let
     * the path follower use its default walking logic. A non-null Movement is
     * inserted into the MovementController ahead of the path follower, which
     * resumes once it finishes.
     *
     * @param from the node this edge starts from
     * @param to   the node this edge leads to
     */
    default Movement createMovement(Node from, Node to) {
        return null;
    }

    /**
     * Whether the path follower must wait until the bot is standing on the
     * ground before dispatching this movement. Ground-based movements
     * (digging, block placement) keep the default; types that take over
     * mid-air or in liquids (e.g. elytra, swimming) can return {@code false}.
     */
    default boolean requiresGroundToDispatch() {
        return true;
    }

    /**
     * Whether the path follower may fall back to plain walking toward the
     * target when {@link #createMovement} returns {@code null}. Walk-like
     * movements return {@code true}. Movements that must place blocks to make
     * the target reachable (bridging, pillaring) return {@code false}: a null
     * result there means the action couldn't run (e.g. the hotbar ran out of
     * blocks), and walking forward would step straight into the gap it was
     * meant to fill, so the follower replans instead.
     */
    default boolean canWalkWhenNoMovement() {
        return true;
    }
}
