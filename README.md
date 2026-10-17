# MovementSync

<!-- Badges -->
<p>
  <a href="https://github.com/huangdihd/MovementSync/releases" target="_blank">
    <img src="https://img.shields.io/github/v/release/huangdihd/MovementSync?style=for-the-badge&label=Release&color=brightgreen" alt="Latest Release">
  </a>
  <a href="https://github.com/huangdihd/MovementSync/issues" target="_blank">
    <img src="https://img.shields.io/github/issues/huangdihd/MovementSync?style=for-the-badge&label=Issues&color=yellow" alt="Issues">
  </a>
  <a href="https://github.com/huangdihd/MovementSync/blob/master/LICENSE" target="_blank">
    <img src="https://img.shields.io/github/license/huangdihd/MovementSync?style=for-the-badge&label=License&color=blue" alt="License">
  </a>
  <a href="https://github.com/huangdihd/MovementSync/stargazers" target="_blank">
    <img src="https://img.shields.io/github/stars/huangdihd/MovementSync?style=for-the-badge&label=Stars&color=ff69b4" alt="Stars">
  </a>
  <a href="https://jitpack.io/#huangdihd/MovementSync" target="_blank">
    <img src="https://img.shields.io/jitpack/version/com.github.huangdihd/MovementSync?style=for-the-badge&label=JitPack&color=b22222" alt="JitPack">
  </a>
</p>

---

> A xinbot plugin to handle bot movement and terrains.  
English / [简体中文](README_CN.md)

---

## Flight in ceiling dimensions

The dimension registry's `has_ceiling` enables roof handling. `min_y` / `height` define world bounds; `logical_height` is not a roof altitude. Loaded collision surfaces determine the roof, without a hard-coded bedrock Y.

- Above the roof, flight retains terrain-clearing cruise. A static goal beneath a roof is replaced by a safe roof landing near its horizontal position; landing reports that the target cannot be reached, ends goto and clears its recovery goal.
- Below the roof, goto walks. There is no automatic roof ascent, drilling or underground corridor flight in this stage. Dangerous falling may deploy rescue gliding only after the existing bounded shared-physics prediction and recovery checks prove it safe; otherwise normal falling handling remains.
- In every dimension, standard flight rules reject trajectories through lava, fire and soul fire. Landing excludes hot supports (including magma blocks and lit campfires) and lava near the standing body. Checks conservatively use whole cells, without modeling ongoing burning or damage cooldowns.

## Quick Start

1. **Download**  
   Get the latest version from [Releases](https://github.com/huangdihd/MovementSync/releases):  
   `MovementSync-[version].jar`

2. **Copy** it to your plugin directory.

3. **Launch** [xinbot](https://github.com/huangdihd/xinbot).

---

## Commands

| Command | Description |
|-----------|-------------|
| `whereami` | Show current position and rotation |
| `jump` | Perform a jump |
| `getblockat` | Get block info at a coordinate |
| `walk` | Walk in a direction for a duration |
| `lookat` | Look at a coordinate |
| `movement` | Control the MovementController (pause/resume/cancel) |
| `entities` | List entities in the world |
| `interactblock` | Interact with a block |
| `interactentity` | Interact with an entity |
| `inventory` | Show inventory |
| `container` | Show current container content |
| `ispassable` | Check if a block is passable |
| `lookatentity` | Look at an entity by ID |
| `goto` | Pathfind to a coordinate; flies there with an elytra when the target is far and rockets are available (`goto <x> <y> <z> walk` forces walking) |
| `elytra` | Elytra flight: `start`, `boost`, `status`, `fly <x> <z> [y]` |
| `follow` | Follow an entity by ID |
| `ride` | Ride an entity or dismount |
| `cansee` | Check line-of-sight to a coordinate |
| `stopmove` | Stop all movement and clear goals |
| `debug` | Print current state for troubleshooting |

---

## Community & Support

- **Issues**: Bug reports and feature requests
- **Discussions**: Share ideas, code snippets, or showcase your plugins based on MovementSync

If you like MovementSync, giving a ⭐ is always appreciated!

---

## License

GPL-3.0-or-later — see [LICENSE](LICENSE) for full text.

- You may use, modify, and distribute the plugin.
- If you distribute modified versions, you must disclose the source and maintain the same license.
- Include copyright and license notices in redistributions.

Made with ❤️ by huangdihd
