package xin.bbtt.commands;

import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.pathfinding.DStarLite;
import xin.bbtt.pathfinding.Node;
import xin.bbtt.movements.PathMovement;

import java.util.List;

public class GotoCommandExecutor extends CommandExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        if (args.length < 3) {
            MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
            return;
        }

        try {
            int tx = Integer.parseInt(args[0]);
            int ty = Integer.parseInt(args[1]);
            int tz = Integer.parseInt(args[2]);

            Vector3d currentPos = MovementSync.Instance.position.get();
            Node start = new Node((int)Math.floor(currentPos.x), (int)Math.floor(currentPos.y), (int)Math.floor(currentPos.z));
            Node goal = new Node(tx, ty, tz);

            // Only check if goal is standable IF the chunk is already loaded.
            // If it's not loaded, we allow the pathfinding to proceed (it will find a partial path).
            if (MovementSync.Instance.getWorld().chunkLoaded(tx >> 4, tz >> 4)) {
                if (!MovementSync.Instance.getWorld().getBlockStateAt(new Vector3d(tx, ty - 1, tz)).isSolid()) {
                    MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.goto.invalid_goal"));
                    return;
                }
            }

            MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.goto.searching", tx, ty, tz));

            DStarLite pathfinder = new DStarLite(start, goal, MovementSync.Instance.getWorld());
            List<Node> path = pathfinder.findPath(5000);

            if (path.size() <= 1) {
                MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.goto.not_found"));
                return;
            }

            MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.goto.success", path.size()));
            MovementSync.Instance.setActiveGoal(new org.joml.Vector3i(tx, ty, tz));
            MovementSync.Instance.getMovementController().addMovement(new PathMovement(path));

        } catch (NumberFormatException e) {
            MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
        } catch (Exception e) {
            MovementSync.Instance.getLogger().error("Error during pathfinding", e);
        }
    }
}
