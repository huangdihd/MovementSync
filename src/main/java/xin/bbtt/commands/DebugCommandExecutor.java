package xin.bbtt.commands;

import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.movement.Movement;

public class DebugCommandExecutor extends CommandExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        // Field diagnostics for the flight rework: "debug shadow on|off" logs the
        // look-ahead solver's shadow decision next to the legacy one; controls are unchanged.
        if (args.length == 2 && args[0].equalsIgnoreCase("shadow")) {
            boolean enabled = args[1].equalsIgnoreCase("on");
            MovementSync.INSTANCE.getFlightScopeRouter().setShadowEnabled(enabled);
            MovementSync.getLogger().info("Shadow flight diagnostics {}", enabled ? "enabled" : "disabled");
            return;
        }
        if (args.length == 2 && args[0].equalsIgnoreCase("localgoto")) {
            if (!args[1].equalsIgnoreCase("on") && !args[1].equalsIgnoreCase("off")) {
                MovementSync.getLogger().info("Usage: debug localgoto on|off");
                return;
            }
            boolean enabled = args[1].equalsIgnoreCase("on");
            MovementSync.INSTANCE.getFlightScopeRouter().setLocalGotoEnabled(enabled);
            MovementSync.INSTANCE.invalidateFlightControls();
            MovementSync.getLogger().info("Local goto controller {}", enabled ? "enabled" : "disabled");
            return;
        }
        if (args.length == 2 && args[0].equalsIgnoreCase("localfollow")) {
            if (!args[1].equalsIgnoreCase("on") && !args[1].equalsIgnoreCase("off")) {
                MovementSync.getLogger().info("Usage: debug localfollow on|off");
                return;
            }
            boolean enabled = args[1].equalsIgnoreCase("on");
            MovementSync.INSTANCE.getFlightScopeRouter().setLocalFollowEnabled(enabled);
            MovementSync.INSTANCE.invalidateFlightControls();
            MovementSync.getLogger().info("Local follow controller {}", enabled ? "enabled" : "disabled");
            return;
        }
        var pos = MovementSync.INSTANCE.position.get();
        var vel = MovementSync.INSTANCE.velocity.get();
        var ctrl = MovementSync.INSTANCE.getMovementController();
        Movement current = ctrl.getCurrentMovement();
        var queue = ctrl.getMovements();

        MovementSync.getLogger().info(LangManager.get("movementsync.command.debug.pos",
                pos.x, pos.y, pos.z));
        var loadedChunks = MovementSync.INSTANCE.getWorld().loadedChunks();
        if (!loadedChunks.isEmpty()) {
            var chunkX = loadedChunks.stream().mapToInt(xin.bbtt.world.World.ChunkPosition::x).summaryStatistics();
            var chunkZ = loadedChunks.stream().mapToInt(xin.bbtt.world.World.ChunkPosition::z).summaryStatistics();
            MovementSync.getLogger().info("Loaded chunks: count={}, x={}..{}, z={}..{}",
                    loadedChunks.size(), chunkX.getMin(), chunkX.getMax(), chunkZ.getMin(), chunkZ.getMax());
        }
        MovementSync.getLogger().info(LangManager.get("movementsync.command.debug.vel",
                vel.x, vel.y, vel.z));
        MovementSync.getLogger().info(LangManager.get("movementsync.command.debug.rotation",
                MovementSync.INSTANCE.yaw.get(), MovementSync.INSTANCE.pitch.get()));
        MovementSync.getLogger().info(LangManager.get("movementsync.command.debug.onground",
                MovementSync.INSTANCE.onGround.get()));
        MovementSync.getLogger().info(LangManager.get("movementsync.command.debug.current",
                current != null ? current.getClass().getSimpleName() : "none"));
        MovementSync.getLogger().info(LangManager.get("movementsync.command.debug.queue", queue.size()));
        MovementSync.getLogger().info(LangManager.get("movementsync.command.debug.goal",
                MovementSync.INSTANCE.getActiveGoal()));
        MovementSync.getLogger().info(LangManager.get("movementsync.command.debug.follow",
                MovementSync.INSTANCE.getFollowTargetId()));
    }
}
