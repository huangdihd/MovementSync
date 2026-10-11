package xin.bbtt.commands;

import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.TabExecutor;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.movements.WalkMovement;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;

public class WalkCommandExecutor extends TabExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        try {
            WalkMovement movement = parseMovement(args);
            MovementSync.INSTANCE.getMovementController().addMovement(movement);
        } catch (IllegalArgumentException error) {
            MovementSync.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
        } catch (Exception error) {
            MovementSync.getLogger().error(LangManager.get("movementsync.command.common.error", error.getMessage()), error);
        }
    }

    static WalkMovement parseMovement(String[] arguments) {
        if (arguments.length < 1 || arguments.length > 3) {
            throw new IllegalArgumentException("Expected direction, optional duration and sprint");
        }
        WalkMovement.Direction direction = WalkMovement.Direction.valueOf(arguments[0].toUpperCase(Locale.ROOT));
        long time = arguments.length >= 2 ? Long.parseLong(arguments[1]) : 1_000L;
        boolean sprint = arguments.length == 3;
        if (sprint && !arguments[2].equalsIgnoreCase("sprint")) {
            throw new IllegalArgumentException("Expected sprint");
        }
        return new WalkMovement(direction, time, sprint);
    }

    @Override
    public List<String> onTabComplete(Command command, String label, String[] args) {
        if (args.length == 1) {
            String prefix = args[0].toUpperCase(Locale.ROOT);
            return Arrays.stream(WalkMovement.Direction.values())
                .map(Enum::name)
                .filter(direction -> direction.startsWith(prefix))
                .toList();
        }
        if (args.length == 3 && "sprint".startsWith(args[2].toLowerCase(Locale.ROOT))) {
            return List.of("sprint");
        }
        return List.of();
    }
}
