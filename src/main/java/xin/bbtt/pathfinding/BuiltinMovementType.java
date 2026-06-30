package xin.bbtt.pathfinding;

import xin.bbtt.movement.Movement;

/**
 * The standard movement types.
 */
public enum BuiltinMovementType implements MovementType {
    /** Walk onto an already standable neighbour. */
    WALK,
    
    /** Walk through an obstacle that must be dug out first. */
    DIG {
        @Override
        public Movement createMovement(Node from, Node to) {
            org.joml.Vector3i[] toCheck = { new org.joml.Vector3i(to.x, to.y, to.z), new org.joml.Vector3i(to.x, to.y + 1, to.z) };
            for (org.joml.Vector3i p : toCheck) {
                xin.bbtt.Block.BlockState state = xin.bbtt.MovementSync.INSTANCE.getWorld().getBlockStateAt(new org.joml.Vector3d(p.x, p.y, p.z));
                if (state.isPassable() || !state.diggable()) continue;

                xin.bbtt.MovementSync.getLogger().info(xin.bbtt.mcbot.LangManager.get("movementsync.pathfinding.obstacle_detected", p.x, p.y, p.z, state.blockName()));
                int toolSlot = xin.bbtt.MovementSync.INSTANCE.getInventoryManager().findBestTool(state.material());
                if (toolSlot != -1) xin.bbtt.MovementSync.INSTANCE.getInventoryManager().switchToSlot(toolSlot);
                return new xin.bbtt.movements.DigBlockMovement(org.cloudburstmc.math.vector.Vector3i.from(p.x, p.y, p.z));
            }
            return null;
        }
    },
    
    /** Jump up one block. */
    JUMP,
    
    /** Drop down to a lower standable block. */
    FALL,
    
    /** Sprint-jump across a 1-2 block gap. */
    GAP_JUMP,
    
    /** Place a block ahead at foot level and walk onto it. */
    BRIDGE {
        @Override
        public Movement createMovement(Node from, Node to) {
            org.joml.Vector3d groundPos = new org.joml.Vector3d(to.x, to.y - 1, to.z);
            xin.bbtt.Block.BlockState ground = xin.bbtt.MovementSync.INSTANCE.getWorld().getBlockStateAt(groundPos);
            if (ground.isSolid()) return null;

            int blockSlot = xin.bbtt.MovementSync.INSTANCE.getInventoryManager().findBlock();
            if (blockSlot == -1) {
                xin.bbtt.MovementSync.getLogger().warn(xin.bbtt.mcbot.LangManager.get("movementsync.pathfinding.no_blocks"));
                return null;
            }

            int[] dx = {0, 0, 0, 0, 1, -1}, dy = {1, -1, 0, 0, 0, 0}, dz = {0, 0, 1, -1, 0, 0};
            org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction[] sides = {
                org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction.DOWN, org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction.UP,
                org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction.NORTH, org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction.SOUTH,
                org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction.WEST, org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction.EAST
            };

            for (int i = 0; i < 6; i++) {
                org.joml.Vector3i neighbor = new org.joml.Vector3i(to.x + dx[i], (to.y - 1) + dy[i], to.z + dz[i]);
                if (!xin.bbtt.MovementSync.INSTANCE.getWorld().getBlockStateAt(new org.joml.Vector3d(neighbor.x, neighbor.y, neighbor.z)).isSolid()) continue;

                xin.bbtt.MovementSync.INSTANCE.getInventoryManager().switchToSlot(blockSlot);
                xin.bbtt.MovementSync.getLogger().info(xin.bbtt.mcbot.LangManager.get("movementsync.pathfinding.bridging", to.x, to.y - 1, to.z));
                return new xin.bbtt.movements.PlaceBlockMovement(org.cloudburstmc.math.vector.Vector3i.from(to.x, to.y - 1, to.z), org.cloudburstmc.math.vector.Vector3i.from(neighbor.x, neighbor.y, neighbor.z), sides[i], true);
            }
            return null;
        }

        @Override
        public boolean canWalkWhenNoMovement() {
            return false;
        }
    },
    
    /** Place a block underfoot while jumping to gain one block of height. */
    PILLAR {
        @Override
        public Movement createMovement(Node from, Node to) {
            org.joml.Vector3d groundPos = new org.joml.Vector3d(to.x, to.y - 1, to.z);
            xin.bbtt.Block.BlockState ground = xin.bbtt.MovementSync.INSTANCE.getWorld().getBlockStateAt(groundPos);
            if (ground.isSolid()) return null;

            int blockSlot = xin.bbtt.MovementSync.INSTANCE.getInventoryManager().findBlock();
            if (blockSlot == -1) {
                xin.bbtt.MovementSync.getLogger().warn(xin.bbtt.mcbot.LangManager.get("movementsync.pathfinding.no_blocks"));
                return null;
            }

            int[] dx = {0, 0, 0, 0, 1, -1}, dy = {1, -1, 0, 0, 0, 0}, dz = {0, 0, 1, -1, 0, 0};
            org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction[] sides = {
                org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction.DOWN, org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction.UP,
                org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction.NORTH, org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction.SOUTH,
                org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction.WEST, org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction.EAST
            };

            for (int i = 0; i < 6; i++) {
                org.joml.Vector3i neighbor = new org.joml.Vector3i(to.x + dx[i], (to.y - 1) + dy[i], to.z + dz[i]);
                if (!xin.bbtt.MovementSync.INSTANCE.getWorld().getBlockStateAt(new org.joml.Vector3d(neighbor.x, neighbor.y, neighbor.z)).isSolid()) continue;

                xin.bbtt.MovementSync.INSTANCE.getInventoryManager().switchToSlot(blockSlot);
                xin.bbtt.MovementSync.getLogger().info(xin.bbtt.mcbot.LangManager.get("movementsync.pathfinding.pillaring", to.x, to.y - 1, to.z));
                xin.bbtt.MovementSync.INSTANCE.getMovementController().insertMovement(new xin.bbtt.movements.PlaceBlockMovement(org.cloudburstmc.math.vector.Vector3i.from(to.x, to.y - 1, to.z), org.cloudburstmc.math.vector.Vector3i.from(neighbor.x, neighbor.y, neighbor.z), sides[i], false));
                return new xin.bbtt.movements.JumpMovement();
            }
            return null;
        }

        @Override
        public boolean canWalkWhenNoMovement() {
            return false;
        }
    }
}