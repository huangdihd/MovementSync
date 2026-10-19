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
| `debug localgoto on\|off` | Toggle local low-altitude goto and initial deployment climb (default on); off restores legacy control. Follow, End and established high-altitude cruise remain unchanged |
| `debug localfollow on\|off` | Toggle local follow control (default on), independently of localgoto; off restores legacy follow control. End and plugin mode remain excluded |
| `debug shadow on\|off` | Toggle flight diagnostics: shadow comparisons on legacy control, prefix/rejection diagnostics on local control |

---

## Flight in ceiling dimensions

Static goto near observed lava uses a known-landing return reserve rather than requiring the entire opposite shore to be loaded. It shares `GlideRange`'s firework resource schedule (inventory order, cooldown, confirmation delay and conservative thrust lifetime). `ReserveMonitor` refreshes in the background about every ten physics ticks and on relevant events; newly loaded supported sites can replace earlier observations. Launch reserves an outward reuse window plus return resources. A planned return previews the complete momentum reversal, climb, return and supported touchdown using shared physics, then executes that control sequence with physical/resource checkpoint checks—not a terminal proof for every candidate each tick. A completed return aborts the original goto instead of walking back toward the unreachable destination; logs distinguish `Planned return` from arrival. End and follow retain their existing policies. The 64-rocket crossing, three-rocket prelaunch rejection and missing-far-shore physical return pass synthetic regression tests; this is not a general safety guarantee or live-server validation, particularly after external corrections, terrain changes or attachment timing divergence.

Field-feedback corrections: a return publication aged 1–8 ticks may be admitted only after replaying its entire stored control sequence from the current state, with unchanged inventory and no attached rocket; checkpoint mismatches still invalidate it. An invalid return retains local control for terrain-checked powered escape instead of an unpowered hold. Over lava, cruise requests an eight-block height margin or restores speed below 0.8 blocks/tick when exposed; supported shore arrival is not kept airborne merely because lava is nearby. Launch-budget geometry may try bounded climbing alternatives when a level excursion is obstructed. These are bounded policies, not guarantees under arbitrary ceilings, cooldowns or attachment delays. The earlier build failed Windows Nether/Grim field tests; these corrections have synthetic coverage but still require field revalidation.

Second field review: periodic monitoring calls `KnownLandingReserve.assess` with a scalable firework-schedule estimate, not an exact touchdown preview. The explicit `ReserveMonitor.resolveReturnEstimate` diagnostic instead reports the accepted exact trajectory's actual rocket consumption; that contract is separate from the conservative reserve estimate. Nearby viable landing sites are assessed independently of distant sites, and `UNKNOWN` is not treated as resource insufficiency. Destination validity uses walking standability at the exact feet coordinates, not the gliding landing-site predicate that requires broad 5×5 support and searches downward; standable narrow ledges and steps are not rejected for lacking a gliding approach. A confirmed unstandable destination is explicitly rejected without dropping airborne control: local safety control and reserve monitoring continue until landing or a resource-triggered return; destination geometry alone does not trigger return. Far corrections through the real position-packet path clear landing memory and invalidate the flight-control epoch, movement-local state and return plans; small corrections retain landing memory, while dimension changes and death clear it. Diagnostics report the actual trigger, observed/current resources, return reserve and continuation margin; return `trajectoryTicks` is separate from aggregate `simulatedSteps` search work. Launch rejection diagnostics distinguish blocked excursion, missing anchor, unproven return, inventory margin and invalid resources. Fifth-field follow-up also continuousizes grounded braking, hazard exit and nongliding handoff yaw using the existing nearest-equivalent-yaw helper. Packet-path regressions bound successive yaw deltas to 180° while preserving physics/use-item rotation equality. This repairs reproduced wrap jumps, but the field logs do not uniquely establish the AimModulo360 cause. Windows Nether/Grim revalidation is still required; the earlier single follow Simulation alert remains unexplained. Compilation/acceptance uses `mvn -o -Dmaven.compiler.release=17 test` (and `package`); Java 21-only APIs are not supported.

### P5f follow-up (in progress; not yet accepted)

The seventh Windows Nether/Grim field report is a severe regression on the approximately 180-block goto to `(175, 60, -32)`, using the prior jar reported as passing 990 tests. The bot log records 16 horizontal collisions at pitch -20, excursions around Y96–115, and `death.attack.flyIntoWall`; 13 collision records have an attached rocket, not all 16. The user also reports two Simulation alerts and aborted testing; those alert strings were not found in the bot log. The sixth report's successful short goto/follow runs therefore do not establish acceptance. Neither sparse motion samples nor null legacy `lastDecision`/prediction fields identify the active control owner: the precise field cause is not uniquely proven, and the recorded terrain/ownership evidence is insufficient for an exact replay.

- **High supported arrival:** when a static, loaded, safe supported goal is below the bot and within 32 blocks horizontally, local goto treats the height overshoot as arrival/descent rather than another cruise leg. It uses the existing checked landing candidates without requesting an extra cruise boost. Below-goal climb, unsupported goals, follow and actual touchdown checks retain their separate policies; this is not permission to land on arbitrary terrain.
- **Bounded missing-control fallback:** valid admitted controls and movement-owned independent landing retain priority. Emergency takeover requires a known hazardous surface (or loaded void boundary) within 12 vertical blocks, low horizontal speed (below 0.8 blocks/tick) or sinking, without overriding confirmed powered ascent. Distant lava and accumulated `fallDistance` alone are no longer hard-fallback climb permissions. The current budget is at most 24 attitude ticks and one rocket request per airborne episode; owner/epoch changes or position corrections do not replenish it. Loaded collision/damage sweeps, inventory, request cooldown and attachment checks still apply. INFO takeover/release records include reason, position, hazard height and owner. This is bounded intervention, not a rescue or landing guarantee.
- **Ceilings and admission review:** local arrival replay and fallback sweeps inspect a no-thrust travel step from the same powered state, because post-travel rocket thrust can mask ceiling-clipped vertical velocity. Powered-prefix admission and revalidation also check pre-thrust ceiling contact. Registered legacy emergency steering now requires a checked sweep. A checked safe touchdown continuation can preserve legitimate descent instead of forcing an unrelated climb; dangerous-descent checks are not removed.
- **Lava cruise and return:** the eight-block lava margin remains a reactive target, not an invariant or survival guarantee under roofs or unavailable thrust. Existing attached thrust can use checked escape steering without a duplicate request. Return retains its early physical flare and stored trajectory checks for fall accumulation and safe supported touchdown; the return audit does not establish that return control caused this field failure. Physics, resource accounting and landing standards are not relaxed.

P5f automated acceptance: two consecutive `mvn -o -Dmaven.compiler.release=17 test` runs and then `package` passed, each reporting 1055 executions, zero failures/errors and one skipped test. This includes inherited test executions, not 1055 unique scenarios. The first integration run exposed Follow regressions; scoped landing-only checks restored the unchanged Follow tests before acceptance. Existing 180/600 physical regressions also pass. These synthetic results do not explain the historical field failure conclusively or verify a live-server fix; Windows Nether/Grim revalidation remains outstanding.

The dimension registry's `has_ceiling` enables roof handling. `min_y` / `height` define world bounds; `logical_height` is not a roof altitude. Loaded collision surfaces determine the roof, without a hard-coded bedrock Y.

- Above the roof, flight retains terrain-clearing cruise. A static goal beneath a roof is replaced by a safe roof landing near its horizontal position; landing reports that the target cannot be reached, ends goto and clears its recovery goal.
- Below the roof, goto asynchronously checks a loaded-terrain geometric corridor and a shared-physics jump/deployment preview; an unloaded destination does not veto a partial route. A rejected grounded deployment can first trigger one bounded inland-launch search: sampled supported sites 8–16 blocks away, up to 16 walking-path attempts, and four launch headings. Each accepted site requires the same strict interruption and delayed-metadata dual-trajectory safety checks plus the applicable route/resource admission (known-return reserve near observed lava). Travel walks there, stops, and revalidates before launch; no safe result still falls back to walking. Other preparation failures log equipment/budget, deployment, corridor-search or worker-status reasons. There is no automatic roof ascent, drilling or block placement. Dangerous falling retains shared-physics rescue checks.
- Ending a static goto flight is not arrival: after ordinary landing, walking retains the original three-dimensional goal; a completed planned reserve return instead ends the request. Partial walking routes replan at their endpoints; unavailable continuations retain the goal and retry every 20 execution ticks. The 200-execution-tick watchdog still reports failure and clears recovery intent when no new node completes; replanning does not reset it. Failure is no longer logged as `arrived`.
- Nongliding entity pursuit uses airborne follow-offset waypoints, with body and predicted sink clearance above the ground endpoint rather than terrain-top cruise altitude. Reaching a waypoint or entering the follow radius does not complete pursuit: it continues in flight until the grounded target slows enough for walking, then selects supported ground near the target and plans the landing from the bot's actual state inside the corridor. Ordinary server position corrections retain active follow intent rather than turning it into stop-and-land; failed flight legs retain a landing owner and resume walking after touchdown. Gliding entities retain the previous pursuit. Emergency landing reuses the ordinary landing-site search with an additional complete-corridor reachability check. Fixed destinations in open dimensions and above roofs retain the original cruise policy.
- The bounded 3D A* searches positions, rejects unknown/colliding/hazardous space, and smooths geometric waypoints using line-of-sight checks. Distant or incompletely loaded destinations return partial routes for rolling worker replanning; there is no 128-block destination cutoff. Execution re-scores yaw/pitch/firework candidates from the actual state using shared glide physics and live corridor clearance. Geometry planning does not prescribe walking inputs; vanilla low-flight execution separately admits bounded physics-tick control sequences. Pursuit scoring integrates predicted tracking error and velocity error (including climbing), while intermediate geometry vertices remain stationary. Embedded pursuit endpoints are lifted to loaded clear air so a target passing through a ridge does not invalidate all planning. Search budgets and physics-derived clearance are documented in the planner; bounded search remains conservative and is not a completeness guarantee.
- Pursuit clearance retains the physics-derived sink reserve above humps and tree collision shapes, including a boosted-reach lookahead beyond the endpoint. If corridor steering is temporarily unavailable, local pursuit previews powered and unpowered airborne controls before resorting to damage-first landing; a harmless touchdown is not itself a valid pursuit action. A lost ground target becomes a trip to its last position, with deployment checks and a fixed-destination walking fallback, rather than immediately ending follow after a failed flight. Lost airborne observations resolve to known supported landing sites so searching ends after landing rather than repeatedly taking off toward the same air coordinate. Landing commitment uses the target velocity filter's remaining-displacement preview and a fixed anchor; pursuit resumes when that preview leaves the keep-distance footprint plus body allowance. All pursuit fallbacks validate the actual trajectory, not merely its phase label. Corridor-selection failures also try freshly validated local pursuit before terminal recovery; a hazard-free recovery result without a steering command does not mean pursuit is impossible. Diagnostics distinguish exhausted recovery budgets from rejected geometry/controls. Low-route selection first replays previously successful recovery controls under a separate bounded budget, rechecking current terrain, inventory and cooldown rather than caching safety. Goto and follow share forward-segment rejoin attempts before terminal/emergency steering. Corridor diagnostics report route projection/direction and observation/proof tick identities, not an untimed one-step error. Low-altitude vanilla controls now carry source physics ticks and navigation ownership, are replay-checked at the input boundary (maximum admission age eight ticks), and use a six-tick selected lease followed by an independently prepared finite touchdown continuation. Expensive preparation stays outside the physics monitor; consumption replays at most six selected steps. This is not a universal safety guarantee: displaced admission does not re-prove the whole terminal trajectory, multi-rocket recovery and plugin-controlled paths are not covered, and loss of all prepared continuations uses explicitly unproved descent. Geometry certification alone remains insufficient. Mode selection uses both recent observed speed and filtered speed to avoid false landing during filter warmup; landing sites outside the target's follow footprint cannot commit a follow landing.
- Planning waits clear old movement/jump input and crouch on ground; low-altitude plans with displaced launch origins are rejected. Low/ hazardous-shore launches prove safe support for no deployment and interruptions during the first five glide ticks. Hazardous-shore launches additionally reject unless a bounded contact/coast preview remains supported and safe with server walking while client glide metadata is delayed by 1, 2 or 4 ticks; each trajectory must stop safely on support, but their positions need not agree. The client retains its glide flag until authoritative metadata arrives; no early flag clearing or assumed Grim tolerance is used. This is a conservative launch rejection gate, not a proof for arbitrary subsequent controls or network delays. Low-altitude goto prepares a loaded (possibly partial) route before jumping; a rejected launch is not retried by the same travel request. Ordinary walking previews the next command followed by neutral coast (up to 80 ticks); hazardous-edge jumps also check this branch. Abort branches now continue through touchdown until stopped; the first deployment tick uses standing height. Hazardous ground handoffs retain crouch, with physically checked reverse steering while glide metadata remains active. States already beyond the stopping boundary are explicitly unproven, not guaranteed rescue. Subsequent external forces or terrain changes are not covered. Ordinary traversal edges also reject after eight execution intervals without measurable 3D motion (placement/mining child waits are excluded), logging the current input and predicted blocking cause instead of waiting for the global watchdog. Rejected path edges are excluded from the current request; full bounded replanning distinguishes budget exhaustion, unloaded frontiers, and exhausted search, logging excluded edges and expanded nodes before UNSAFE_ROUTE. Walking also excludes body contact with fire/soul fire. Failure/cancel clears old inputs and independently takes over landing or bounded ground escape, including nongliding airborne touchdown/coast risks. Independent landing retains control after glide metadata clears and across repeated cancellation; correction/metadata diagnostics distinguish authoritative corrections from explicit stops. Ground escape requires a hazard-free supported stop after escape; unproven exits warn instead of claiming rescue. Enclosed hazards, remaining health, and extinguishing residual burning are not modeled. Low flight rejects horizontally clipped/out-of-corridor candidates; rescue may prefer a proven target-nearer landing when resources permit, retaining the original fallback otherwise. Terminal low flight adds braking controls and suppresses fresh rockets, with cooldown also enforced at execution. Logs distinguish inventory from currently usable rockets. Launch lifecycle diagnostics use INFO; no-progress WARN events include nodes, velocity, grounded state and nearby support blocks.
- Live gliding and flight previews share vanilla float-sized body dimensions: half-width `0.6F / 2`, height `0.6F` (widened to double), including support and hazard queries; there is no `.299` glide collision inset.
- Low-altitude hazard reserves account for remaining rockets, cooldown and attachment; horizontally stalled branches may also spend available rockets rather than requiring unpowered sea crossings. This reuses the End's `GlideRange` minimum rocket lifetime, loaded-site `LandingSearch` and `LandingMemory`. Range alone cannot replace tick-by-tick roofed collision/hazard validation. Candidate rocket consumption must retain recovery capacity; unproven routes are rejected, and already unrecoverable states cannot be guaranteed rescue. Recovery checks rank candidates first and reuse control-search hints, never cached safety conclusions: each hint is replayed to terminal landing. Each selector invocation or six-step safety validation permits at most 24,000 recovery physics steps, with landing search limited to 512 visited nodes. This is not a whole-movement-tick budget; terminal proofs still execute synchronously, pending safe worker handoff with terrain versions and complete control witnesses.
- Walking speed at soul-sand/soul-soil boundaries uses the supporting block's X/Z rather than only the feet-center column. Support selection uses the thin slice beneath the feet, nearest block center and vanilla tie ordering, preserving soul sand's 14/16 collision height and soul soil's ordinary speed.
- Death and respawn clear flight state and old actions, quarantining gliding metadata until a new deployment request succeeds. This prevents previous-life metadata from starting safe landing; a new login still permits airborne reconnect adoption.
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
