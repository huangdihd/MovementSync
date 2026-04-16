package xin.bbtt.commands;

import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.jline.utils.AttributedStyle;
import xin.bbtt.Entity.Entity;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.TabHighlightExecutor;
import xin.bbtt.mcbot.LangManager;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class EntitiesCommandExecutor extends TabHighlightExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        Map<Integer, Entity> entities = MovementSync.Instance.getWorld().getEntities();
        String filter = args.length > 0 ? args[0].toLowerCase() : null;

        int count = 0;
        for (Entity e : entities.values()) {
            if (filter == null || e.getType().name().toLowerCase().contains(filter)) {
                count++;
            }
        }

        MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.entities.header", count));
        for (Map.Entry<Integer, Entity> entry : entities.entrySet()) {
            Entity e = entry.getValue();
            if (filter != null && !e.getType().name().toLowerCase().contains(filter)) {
                continue;
            }
            String posStr = String.format("(%.1f, %.1f, %.1f)", e.getPosition().x, e.getPosition().y, e.getPosition().z);
            MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.entities.entry_with_pos", entry.getKey(), e.getType(), posStr));
        }
    }

    @Override
    public List<String> onTabComplete(Command command, String label, String[] args) {
        if (args.length == 1) {
            String prefix = args[0].toLowerCase();
            return Arrays.stream(EntityType.values())
                    .map(Enum::name)
                    .map(String::toLowerCase)
                    .filter(name -> name.startsWith(prefix))
                    .collect(Collectors.toList());
        }
        return new ArrayList<>();
    }

    @Override
    public AttributedStyle[] onHighlight(Command command, String label, String[] args) {
        AttributedStyle[] styles = new AttributedStyle[args.length];
        if (args.length > 0) {
            styles[0] = new AttributedStyle().foreground(AttributedStyle.CYAN);
        }
        return styles;
    }
}
