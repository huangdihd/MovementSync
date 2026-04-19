package xin.bbtt.commands;

import org.geysermc.mcprotocollib.protocol.data.game.entity.player.InteractAction;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;
import xin.bbtt.mcbot.command.SubCommandExecutor;
import xin.bbtt.movements.InteractEntityMovement;

import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class RideCommandExecutor extends SubCommandExecutor {
    public RideCommandExecutor() {
        registerSubCommand("dismount", new CommandExecutor() {
            @Override
            public void onCommand(Command command, String label, String[] args) {
                if (MovementSync.Instance.isRiding()) {
                    MovementSync.Instance.setRidingSneak(true);
                    MovementSync.Instance.getLogger().info("Dismounting (sending sneak input)...");
                    // Reset sneak after a short delay or in next tick if needed, 
                    // but usually sending it once is enough to trigger the server-side dismount.
                    MovementSync.Instance.movementService.schedule(() -> MovementSync.Instance.setRidingSneak(false), 200, java.util.concurrent.TimeUnit.MILLISECONDS);
                } else {
                    MovementSync.Instance.getLogger().info("Not riding any entity.");
                }
            }
        });
    }

    @Override
    public void onCommand(Command command, String label, String[] args) {
        if (args.length == 0) {
            onNoSubCommand(command, label);
            return;
        }

        if (args[0].equalsIgnoreCase("dismount")) {
            super.onCommand(command, label, args);
            return;
        }

        try {
            int entityId = Integer.parseInt(args[0]);
            MovementSync.Instance.getMovementController().addMovement(new InteractEntityMovement(entityId, InteractAction.INTERACT));
            MovementSync.Instance.getLogger().info("Attempting to ride entity " + entityId);
        } catch (NumberFormatException e) {
            onNoSubCommand(command, label);
        }
    }

    @Override
    public List<String> onTabComplete(Command command, String label, String[] args) {
        if (args.length == 1) {
            List<String> suggestions = MovementSync.Instance.getWorld().getEntities().keySet().stream()
                    .map(String::valueOf)
                    .filter(s -> s.startsWith(args[0]))
                    .collect(Collectors.toList());
            if ("dismount".startsWith(args[0].toLowerCase())) {
                suggestions.add("dismount");
            }
            return suggestions;
        }
        return Collections.emptyList();
    }

    @Override
    protected void onNoSubCommand(Command command, String label) {
        MovementSync.Instance.getLogger().info("Usage: " + command.getUsage());
    }
}
