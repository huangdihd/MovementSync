package xin.bbtt.movements;

import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;
import xin.bbtt.physics.PlayerInput;

import java.util.Objects;

public class WalkMovement extends Movement {
    public enum Direction {
        FRONT(null), BACK(null), LEFT(null), RIGHT(null),
        NORTH(180f), SOUTH(0f), WEST(90f), EAST(-90f);

        private final Float yaw;

        Direction(Float yaw) {
            this.yaw = yaw;
        }
    }

    private final Direction direction;
    private final long time;
    private final boolean sprint;
    private PlayerInput acquiredInput;

    public WalkMovement(Direction direction, long time) {
        this(direction, time, false);
    }

    public WalkMovement(Direction direction, long time, boolean sprint) {
        this.direction = Objects.requireNonNull(direction, "direction");
        if (time < 0) throw new IllegalArgumentException("Walk duration must not be negative");
        this.time = time;
        this.sprint = sprint;
    }

    @Override
    public void init() {
        if (acquiredInput != null) return;
        if (direction.yaw != null) MovementSync.INSTANCE.setYawToward(direction.yaw);
        PlayerInput current = MovementSync.INSTANCE.getInput();
        boolean forward = direction == Direction.FRONT || direction.yaw != null;
        boolean backward = direction == Direction.BACK;
        boolean left = direction == Direction.LEFT;
        boolean right = direction == Direction.RIGHT;
        acquiredInput = new PlayerInput(
            forward && !current.forward(), backward && !current.backward(),
            left && !current.left(), right && !current.right(), false, false,
            sprint && !current.sprint());
        MovementSync.INSTANCE.setInput(new PlayerInput(
            current.forward() || forward, current.backward() || backward,
            current.left() || left, current.right() || right,
            current.jump(), current.sneak(), current.sprint() || sprint));
    }

    @Override
    public void onTick() {
    }

    @Override
    public long getTime() {
        return time;
    }

    @Override
    public void onStop() {
        if (acquiredInput == null) return;
        PlayerInput current = MovementSync.INSTANCE.getInput();
        // Release only keys acquired here; jump and sneak belong to other input producers.
        MovementSync.INSTANCE.setInput(new PlayerInput(
            current.forward() && !acquiredInput.forward(),
            current.backward() && !acquiredInput.backward(),
            current.left() && !acquiredInput.left(),
            current.right() && !acquiredInput.right(),
            current.jump(), current.sneak(), current.sprint() && !acquiredInput.sprint()));
        acquiredInput = null;
    }

    @Override
    public void onPause() {
        onStop();
    }

    @Override
    public void onResume() {
        init();
    }
}
