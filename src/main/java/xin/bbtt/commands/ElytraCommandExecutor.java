package xin.bbtt.commands;

import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.inventory.ItemRegistry;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;
import xin.bbtt.mcbot.command.SubCommandExecutor;
import xin.bbtt.movements.ElytraTakeoffMovement;

public class ElytraCommandExecutor extends SubCommandExecutor {
    public ElytraCommandExecutor() {
        registerSubCommand("start", new CommandExecutor() {
            @Override
            public void onCommand(Command command, String label, String[] args) {
                MovementSync movementSync = MovementSync.INSTANCE;
                if (movementSync.isRiding()) {
                    MovementSync.getLogger().info(LangManager.get("movementsync.command.elytra.start.riding"));
                    return;
                }
                if (movementSync.isGliding()) {
                    MovementSync.getLogger().info(LangManager.get("movementsync.command.elytra.start.gliding"));
                    return;
                }
                ItemStack[] inventory = movementSync.getInventoryManager().getInventory();
                ItemStack chest = inventory != null && inventory.length > 6 ? inventory[6] : null;
                ItemRegistry.ItemEntry item = chest == null ? null : ItemRegistry.Instance.getItem(chest.getId());
                if (chest == null || chest.getAmount() <= 0 || item == null || !"elytra".equals(item.getName())) {
                    MovementSync.getLogger().info(LangManager.get("movementsync.command.elytra.start.no_elytra"));
                    return;
                }
                movementSync.getMovementController().addMovement(new ElytraTakeoffMovement());
            }
        });
        registerSubCommand("boost", new CommandExecutor() {
            @Override
            public void onCommand(Command command, String label, String[] args) {
                if (!MovementSync.INSTANCE.useFirework()) {
                    MovementSync.getLogger().info(LangManager.get("movementsync.command.elytra.boost.failed"));
                }
            }
        });
        registerSubCommand("status", new CommandExecutor() {
            @Override
            public void onCommand(Command command, String label, String[] args) {
                MovementSync movementSync = MovementSync.INSTANCE;
                Vector3d position = new Vector3d(movementSync.position.get());
                Vector3d velocity = new Vector3d(movementSync.velocity.get());
                MovementSync.getLogger().info(LangManager.get("movementsync.command.elytra.status",
                        movementSync.isGliding(), position.x, position.y, position.z,
                        velocity.x, velocity.y, velocity.z, Math.hypot(velocity.x, velocity.z),
                        movementSync.yaw.get(), movementSync.pitch.get(), movementSync.hasAttachedFirework()));
            }
        });
    }

    @Override
    protected void onNoSubCommand(Command command, String label) {
        MovementSync.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
    }
}
