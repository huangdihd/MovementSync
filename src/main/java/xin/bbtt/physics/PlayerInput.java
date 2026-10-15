package xin.bbtt.physics;

/** Immutable keys sampled once at the beginning of a player tick. */
public record PlayerInput(
        boolean forward, boolean backward, boolean left, boolean right,
        boolean jump, boolean sneak, boolean sprint) {
    public static final PlayerInput NONE = new PlayerInput(false, false, false, false, false, false, false);

    public PlayerInput withJump(boolean pressed) {
        return new PlayerInput(forward, backward, left, right, pressed, sneak, sprint);
    }
}
