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

## Flight in ceiling dimensions

The dimension registry's `has_ceiling` enables roof handling. `min_y` / `height` define world bounds; `logical_height` is not a roof altitude. Loaded collision surfaces determine the roof, without a hard-coded bedrock Y.

- Above the roof, flight retains terrain-clearing cruise. A static goal beneath a roof is replaced by a safe roof landing near its horizontal position; landing reports that the target cannot be reached, ends goto and clears its recovery goal.
- Below the roof, goto asynchronously checks a loaded-terrain geometric corridor and a shared-physics jump/deployment preview; failed preparation falls back to walking. There is no automatic roof ascent, drilling or block placement. Dangerous falling retains shared-physics rescue checks.
- Nongliding entity pursuit uses airborne follow-offset waypoints, with body and predicted sink clearance above the ground endpoint rather than terrain-top cruise altitude. Reaching a waypoint or entering the follow radius does not complete pursuit: it continues in flight until the grounded target slows enough for walking, then selects supported ground near the target and plans the landing from the bot's actual state inside the corridor. Ordinary server position corrections retain active follow intent rather than turning it into stop-and-land; failed flight legs retain a landing owner and resume walking after touchdown. Gliding entities retain the previous pursuit. Emergency landing reuses the ordinary landing-site search with an additional complete-corridor reachability check. Fixed destinations in open dimensions and above roofs retain the original cruise policy.
- The bounded 3D A* searches positions, rejects unknown/colliding/hazardous space, and smooths geometric waypoints using line-of-sight checks. Distant or incompletely loaded destinations return partial routes for rolling worker replanning; there is no 128-block destination cutoff. Execution re-scores yaw/pitch/firework candidates from the actual state using shared glide physics and live corridor clearance. There is no per-tick input replay. Pursuit scoring integrates predicted tracking error and velocity error (including climbing), while intermediate geometry vertices remain stationary. Embedded pursuit endpoints are lifted to loaded clear air so a target passing through a ridge does not invalidate all planning. Search budgets and physics-derived clearance are documented in the planner; bounded search remains conservative and is not a completeness guarantee.
- Live gliding and flight previews share vanilla float-sized body dimensions: half-width `0.6F / 2`, height `0.6F` (widened to double), including support and hazard queries; there is no `.299` glide collision inset.
- In every dimension, standard flight rules reject trajectories through lava, fire and soul fire. Landing excludes hot supports (including magma blocks and lit campfires) and lava near the standing body. Checks conservatively use whole cells, without modeling ongoing burning or damage cooldowns.

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
