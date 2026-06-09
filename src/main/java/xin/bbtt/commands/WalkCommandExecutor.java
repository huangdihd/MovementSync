package xin.bbtt.commands;

import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.TabExecutor;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.movements.WalkMovement;
import xin.bbtt.world.Direction;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class WalkCommandExecutor extends TabExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        if (args.length == 0) return;
        if (args.length == 1) {
            args = new String[]{args[0], "1000"};
        }
        Vector3d velocity;
        try {
            switch (args[0].toUpperCase()) {
                case "FRONT" ->
                        velocity = Direction.getUnitVectorByYaw(MovementSync.INSTANCE.yaw.get()).mul(MovementSync.movementSpeed);
                case "LEFT" -> {
                    Vector3d right = new Vector3d();
                    Direction.getUnitVectorByYaw(MovementSync.INSTANCE.yaw.get()).cross(Direction.UP.getUnitVector(), right).normalize();
                    velocity = right.negate().mul(MovementSync.movementSpeed);
                }
                case "BACK" ->
                        velocity = Direction.getUnitVectorByYaw(MovementSync.INSTANCE.yaw.get()).negate().mul(MovementSync.movementSpeed);
                case "RIGHT" -> {
                    Vector3d right = new Vector3d();
                    Direction.getUnitVectorByYaw(MovementSync.INSTANCE.yaw.get()).cross(Direction.UP.getUnitVector(), right).normalize();
                    velocity = right.mul(MovementSync.movementSpeed);
                }
                default -> {
                    Direction direction = Direction.valueOf(args[0].toUpperCase());
                    if (direction.getUnitVector().y() != 0) return;
                    velocity = direction.getVector(MovementSync.movementSpeed);
                }
            }
            long time = Long.parseLong(args[1]);
            MovementSync.INSTANCE.getMovementController().addMovement(new WalkMovement(velocity, time));
        } catch (IllegalArgumentException e) {
            // Bad direction or duration argument.
            MovementSync.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
        } catch (Exception e) {
            MovementSync.getLogger().error(LangManager.get("movementsync.command.common.error", e.getMessage()), e);
        }
    }

    @Override
    public List<String> onTabComplete(Command command, String label, String[] args) {
        if (args.length == 1) {
            List<String> directions = Arrays.stream(Direction.values())
                    .filter(d -> d.getUnitVector().y() == 0)
                    .map(Direction::toString)
                    .collect(Collectors.toList());
            directions.add("FRONT");
            directions.add("LEFT");
            directions.add("BACK");
            directions.add("RIGHT");
            return directions.stream().filter(s -> s.toLowerCase().startsWith(args[0].toLowerCase())).collect(Collectors.toList());
        }
        return List.of();
    }
}
