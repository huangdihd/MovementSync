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
        var pos = MovementSync.INSTANCE.position.get();
        var vel = MovementSync.INSTANCE.velocity.get();
        var ctrl = MovementSync.INSTANCE.getMovementController();
        Movement current = ctrl.getCurrentMovement();
        var queue = ctrl.getMovements();

        MovementSync.getLogger().info(LangManager.get("movementsync.command.debug.pos",
                pos.x, pos.y, pos.z));
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
