package xin.bbtt.commands;

import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;
import xin.bbtt.mcbot.command.SubCommandExecutor;
import xin.bbtt.movements.ElytraFlyMovement;
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
                if (!movementSync.hasEquippedElytra()) {
                    MovementSync.getLogger().info(LangManager.get("movementsync.command.elytra.start.no_elytra"));
                    return;
                }
                if (!movementSync.getInventoryManager().canStartFlight(0)) {
                    MovementSync.getLogger().info("Insufficient prepared elytra durability for takeoff");
                    return;
                }
                movementSync.getMovementController().addMovement(new ElytraTakeoffMovement());
            }
        });
        registerSubCommand("fly", new CommandExecutor() {
            @Override
            public void onCommand(Command command, String label, String[] args) {
                MovementSync movementSync = MovementSync.INSTANCE;
                if (args.length < 2 || args.length > 3) {
                    MovementSync.getLogger().info(LangManager.get("movementsync.command.elytra.fly.usage"));
                    return;
                }
                Vector3d target;
                try {
                    double x = Double.parseDouble(args[0]);
                    double z = Double.parseDouble(args[1]);
                    double y = args.length == 3 ? Double.parseDouble(args[2])
                            : movementSync.getWorld().findHighestCollisionTop(x - 0.3,
                                    movementSync.getWorld().getMinWorldY(), z - 0.3,
                                    x + 0.3, movementSync.getWorld().getMaxWorldY(), z + 0.3)
                                    .orElse(movementSync.getWorld().getMinWorldY() + 4);
                    target = new Vector3d(x, y, z);
                    if (!target.isFinite()) throw new NumberFormatException("Coordinates must be finite");
                } catch (NumberFormatException error) {
                    MovementSync.getLogger().info(LangManager.get("movementsync.command.elytra.fly.usage"));
                    return;
                }
                if (movementSync.isRiding()) {
                    MovementSync.getLogger().info(LangManager.get("movementsync.command.elytra.start.riding"));
                    return;
                }
                if (!movementSync.hasEquippedElytra()) {
                    MovementSync.getLogger().info(LangManager.get("movementsync.command.elytra.start.no_elytra"));
                    return;
                }
                if (movementSync.onGround.get() && movementSync.countHotbarFireworks() == 0) {
                    MovementSync.getLogger().info(LangManager.get("movementsync.command.elytra.fly.no_fireworks"));
                    return;
                }
                if (!movementSync.getInventoryManager().canStartFlight(Math.hypot(
                        target.x - movementSync.position.get().x, target.z - movementSync.position.get().z))) {
                    MovementSync.getLogger().info("Insufficient prepared elytra durability for flight");
                    return;
                }
                // Claim without a ground goal so old follow/goto requests cannot resume after flight.
                long generation = movementSync.beginStaticNavigationRequest(null);
                boolean published = false;
                try {
                    published = movementSync.addMovementIfNavigationRequestCurrent(
                            generation, new ElytraFlyMovement(target, generation));
                    if (published) {
                        MovementSync.getLogger().info(LangManager.get("movementsync.command.elytra.fly.started",
                                target.x, target.y, target.z));
                    }
                } finally {
                    if (!published) movementSync.abandonNavigationRequest(generation);
                }
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
