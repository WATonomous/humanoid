---
name: isaac-harness
description: Use whenever driving IsaacLab/Isaac Sim scenes in this repo remotely — no physical access to the monitor (Claude Code Remote Control, SSH, headless box) — for tasks like building/checking a scene, spawning objects or this repo's robots, taking screenshots to show the user, or verifying scene geometry (positions, overlaps, clearances) programmatically. Also read before touching tools/isaac_harness/ itself. Triggers on "show me the scene", "take a screenshot of isaac sim", "build a scene with X", "spawn the robot", "check if X is on the table", or any remote IsaacLab iteration.
---

# isaac_harness

`tools/isaac_harness/` is this repo's toolkit for driving IsaacLab/Isaac Sim scenes when
there's no physical access to the monitor. Built and hardened over one long session —
every gotcha below cost real time to discover; don't rediscover them.

## Pick the right tool first

| Need | Use |
|---|---|
| One screenshot, single check | `isaac_screenshot.sh` |
| Several camera angles of one build, single check | `isaac_multiview_screenshot.sh` (GUI/xwd) or `isaac_headless_multiview.sh` (faster, no display) |
| Building a scene incrementally, checking positions, iterating on placement, anything with >1-2 round trips | **`isaac_session.sh` + `isaac_session_client.py`** — a persistent daemon. Relaunching Isaac Sim costs ~15-40s every time; against a running daemon each command costs well under 2s. |

Full command reference and usage examples: `tools/isaac_harness/README.md`. This skill is
the "how to not get burned" layer on top of that, not a replacement for it.

## The daemon workflow (the common case)

```bash
tools/isaac_harness/isaac_session.sh start          # ~20-30s, once
python3 tools/isaac_harness/isaac_session_client.py spawn_primitive --name X ...
python3 tools/isaac_harness/isaac_session_client.py spawn_bimanual_arm --name Robot ...
python3 tools/isaac_harness/isaac_session_client.py screenshot --eye ... --target ... --out shot.png
tools/isaac_harness/isaac_session.sh stop           # see shutdown timing note below
```

There's a saved reference scene at `tools/isaac_harness/scenes/bimanual_vial_rack.sh`
(table + this repo's robot + vial rack + 3 vials) — reuse or extend it rather than
re-typing spawn commands from scratch.

## Hard rules

- **Never `SIGKILL` (`kill -9`) an Isaac Sim process without the user's explicit
  go-ahead for that specific process, every time.** SIGKILL gives it no chance to
  release its GPU/graphics context. The blast radius has ranged from "next launch
  segfaults instantly" to "needs a full machine reboot to recover." Always try
  `kill -SIGINT` (same as Ctrl+C) or plain `kill` (SIGTERM) first, or the daemon's own
  `shutdown` command. `isaac_session.sh` already does this before relaunching.
- **Never pass `--experience isaacsim.exp.full.kit`.** That's for an unrelated
  cloud/pip-only Isaac Sim setup and causes a deterministic native segfault
  (`omni.graph.core.plugin.so`) on this repo's local Docker install. Omit the flag —
  `isaaclab.sh` picks the correct local experience on its own.
- **Check the host's actual `$DISPLAY` before any GUI-mode launch.** It is NOT
  reliably `:0` — this box can boot into Wayland (`:0`) or a plain X11 session (`:1`),
  and it changes across reboots. Verify with `echo $DISPLAY` / `w` first. Headless mode
  sidesteps this entirely — prefer it when a live window isn't specifically needed.
- **Graceful shutdown can legitimately take minutes on a heavy, long-lived daemon
  session** (many spawned objects, an articulation, a long command history) — this is
  confirmed, not a guess: the daemon's shutdown *logic* was code-reviewed and found
  correct, and `simulation_app.close()` was still observed pumping ~65% GPU load 3+
  minutes after accepting the shutdown command.
  **Standing exception to the SIGKILL rule above, for this specific daemon only:**
  the user has explicitly authorized SIGKILLing `isaac_session_daemon.py` (not any
  other Isaac Sim process) after a ~15-30s wait when graceful shutdown hangs like
  this — no need to ask each time. It's been repeated several times and always
  relaunches cleanly afterward. Always verify with a fresh `isaac_session.sh start`
  that it does come back up clean; if a launch ever crashes after this, stop and go
  back to asking before further force-kills, since that would mean the pattern
  changed. This exception does NOT extend to one-shot scripts or GUI launches — those
  still need per-instance authorization.

## Use `bbox`/`query`/`distance` to answer questions, not screenshots + guessing

The single biggest workflow mistake made building this: repeatedly guessing a
parameter (e.g. "is the ground plane in the right place?"), taking a screenshot,
eyeballing it, guessing again. This wastes many round trips and the guesses can still
be wrong in ways that aren't obvious from a render angle.

Instead: **query the actual geometry and compute the answer directly.**

Example — placing a ground plane flush under a robot's base instead of trial-and-error:
```bash
python3 isaac_session_client.py bbox --name Robot
# {"min": [-0.30, -0.38, -1.1997], "max": [0.35, 0.38, 0.30], ...}
# → ground Z should be -1.1997, not a guessed round number
```
This found a real bug: a previously "looks fine" screenshot was actually hiding the
ground plane cutting 0.2m into the robot's base — invisible at that camera angle,
obvious as soon as the actual number was checked.

**Known limitation to work around, not ignore:** `bbox` reads raw USD stage
transforms, which do NOT reliably sync from the physics tensor/Fabric layer every
step. For an object that's actively moving/falling under physics, `bbox` can silently
return a stale, spawn-time position — `query()`'s tensor-sourced pose is authoritative
for a moving tracked object instead. The daemon flags this itself: any `bbox` response
for a tracked *dynamic* (non-kinematic) object includes a `"warning"` field saying
exactly this — check for it, and prefer `query()` when it's present and the object may
have moved. Static/kinematic objects (tables, ground, anything that hasn't been
stepped) don't carry this risk.

`overlap` (AABB overlap, boolean) and `distance` (center-to-center) exist to answer
"is X actually on Y" / "are these interpenetrating" programmatically — use them instead
of a screenshot + eyeball judgment call when the question is that concrete.

## Robot-specific

- **This repo's robot is `pioneer_bimanual_arm.usd`.** If it looks like a single
  6-DOF arm rather than two arms, or looks like an old/wrong model, `git pull` — the
  canonical asset and its config (`Teleop/keyboard_based_teleoperation/bimanual_arm_cfg.py`)
  have moved before (deleted+replaced in commit `950b86b8`, "Adding the new URDF and
  USD files... Removing unused USD/URDF files").
- **Spawn it with `spawn_bimanual_arm`, not generic `spawn_usd --articulation`.**
  The generic command builds a bare `ArticulationCfg` with no actuators and throws
  `TypeError: Missing values detected ... - actuators`. `spawn_bimanual_arm` uses the
  real, actuator-complete `BIMANUAL_ARM_CFG` instead.

## Object-spawning gotchas

- `spawn_primitive --static` means *kinematic* (still a rigid body, just not
  physics-driven), not "no rigid body at all" — `RigidObject` requires
  `USD RigidBodyAPI` on the prim regardless.
- Spawning a rigid object exactly at its expected resting height risks a small
  initial interpenetration with whatever it's resting on, which can trigger a violent
  PhysX correction impulse (observed: a vial spawned at table-top+0.05 tipped over and
  flew within 5 physics steps; +0.02 extra clearance fixed it). Spawn slightly above
  resting height and let physics settle, don't spawn exactly at it.

## When to ask vs. proceed

Ask before forcing a stuck shutdown (see hard rules). Otherwise, act — spawn, step,
query, screenshot freely; none of that is destructive or hard to undo. Multi-view
screenshots from several angles are usually worth the extra couple of calls when
judging spatial correctness (a single angle can hide a problem, as the ground-plane
bug above shows) — don't stop at one shot if the question is "is this actually right."
