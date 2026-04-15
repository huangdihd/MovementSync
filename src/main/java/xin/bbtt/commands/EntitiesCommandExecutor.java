package xin.bbtt.commands;

import xin.bbtt.Entity.Entity;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;
import xin.bbtt.mcbot.LangManager;

import java.util.Map;

public class EntitiesCommandExecutor extends CommandExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        Map<Integer, Entity> entities = MovementSync.Instance.getWorld().getEntities();
        MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.entities.header", entities.size()));
        for (Map.Entry<Integer, Entity> entry : entities.entrySet()) {
            Entity e = entry.getValue();
            String posStr = String.format("(%.1f, %.1f, %.1f)", e.getPosition().x, e.getPosition().y, e.getPosition().z);
            MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.entities.entry_with_pos", entry.getKey(), e.getType(), posStr));
        }
    }
}
