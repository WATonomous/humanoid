# Handoff: VR Teleop + Pink IK (exported from the Isaac Lab session)

> **Purpose of this file:** carry the context/knowledge worked out in a separate Cursor
> session (in `~/robotics/IsaacLab-3.0`) into this repo. It is **self-contained** — it does
> not depend on that session's memory.
>
> **How to use in a fresh Cursor session here:** open this repo, then tell the agent
> *"read `TELEOP_PINK_IK_HANDOFF.md` before we start."* (Optionally copy it into
> `.cursor/rules/` so it auto-loads.)
>
> **NOTE / TODO (user will fill in):** the specifics of THIS repo — our actual robot/arm
> (NOT the SO-100/SO-101), its USD/URDF, joint names, EE links, and how `autonomy/teleop`
> + `autonomy/simulation` are organized — will be described separately. Do **not** assume
> any of that from the file paths; treat repo details as unknown until provided.

---

## Goal
Bring two things, proven working in the Isaac Lab session, into this repo for **our own arm**:
1. **VR headset teleop** of an Isaac Lab/Isaac Sim scene to a **Meta Quest** via NVIDIA **CloudXR.js** (browser client).
2. **Pink IK** (QP-based differential inverse kinematics) driving our arm from the teleop pose targets.

Below is (A) the working teleop stack + exact gotchas, and (B) how Isaac Lab wires Pink IK so we can replicate it. Source of truth for both is `~/robotics/IsaacLab-3.0` (Isaac Sim 6.0 + Isaac Lab 3.0 `release/3.0.0-beta2`).

> **APPROACH — do NOT build from scratch.** Reuse Isaac Lab's existing machinery as-is:
> the `teleop_se3_agent.py` script, the `isaacteleop`/`isaaclab_teleop` CloudXR pipeline, and
> the `PinkIKController` + `PinkInverseKinematicsAction`. The only thing we author for this repo
> is a **new Isaac Lab env config for OUR setup**:
> - **our arm(s)** — our robot articulation (likely **bimanual** → one IK FrameTask + one EE link
>   *per arm*; `action_dim = #FrameTasks × 7 (+ hand joints)`), with our arm's USD/URDF, joint
>   names, base link, and EE link names;
> - **our environment/scene** — our own world (table, objects, props, lighting), **not** the stock
>   Franka cube-stack scene;
> - an `isaac_teleop` (`IsaacTeleopCfg`) block + a `PinkInverseKinematicsActionCfg` /
>   `PinkIKControllerCfg` pointed at the above.
>
> No new teleop transport, no new IK solver — just a new env/scene config + our robot asset(s)
> plugged into the existing pipeline.

---

## PART A — Working Quest teleop stack (CloudXR.js)

### Why the stack versions matter
- The Quest/browser CloudXR.js path **cannot run on Isaac Sim 4.5** — it needs CloudXR **6.x in-process** + **Isaac Teleop**, which only exist in **Isaac Sim 6.0 / Isaac Lab 3.0**. Sim 4.5 segfaults at the SDP/stream-offer step (this is the source of all the `0xC0F2xxxx` errors).
- Isaac Lab must be on the **`release/3.0.0-beta2`** branch, **not** the `v3.0.0-beta` tag (beta1). beta1 + Sim 6.0 GA = every task crashes at env init with `Exception: Failed to get DOF velocities from backend` (numpy→torch backend switch invalidates the physics view; `--device cpu` does NOT fix it). beta2 fixes it.

### Working stack
| Component | Version / location |
|---|---|
| OS / GPU | Ubuntu 22.04, X11; RTX 2070 **8 GB** (the binding constraint), driver 575.x, CUDA 12.x |
| Isaac Sim | 6.0.0 binary (`~/robotics/isaacsim-6.0`), ran `post_install.sh` |
| Isaac Lab | 3.0, branch `release/3.0.0-beta2` (`~/robotics/IsaacLab-3.0`), `_isaac_sim`→`isaacsim-6.0` |
| isaacteleop | 1.3.x (bundled CloudXR runtime 6.2.0), installed by `./isaaclab.sh -i` |
| CloudXR.js client | hosted, URL path **`release-1.3.x`** (must match isaacteleop version) |

### Network (campus wifi blocks device-to-device)
- CloudXR.js needs **3 ports**: `49100/tcp` (signaling), `48322/tcp` (WSS proxy/HTTPS), **`47998/udp` (video)**.
- `adb reverse` is **TCP-only** → signaling connects but **video is black** (UDP can't pass).
- **`gnirehtet`** (Genymobile, USB reverse-tether) tunnels **all TCP + UDP** through the PC → works. Quest reaches the PC by its **real IP** (not `localhost`). Install: `adb install -r -g gnirehtet.apk`; run `./gnirehtet run`; approve the VPN prompt on the headset.

### Run procedure (the working invocation)
```bash
# Terminal 1 — USB tunnel (approve VPN on headset once)
cd <gnirehtet>/gnirehtet-rust-linux64 && ./gnirehtet run

# Terminal 2 — teleop (headless XR auto-starts; CPU physics frees VRAM)
cd ~/robotics/IsaacLab-3.0
./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
  --task <YOUR_ISAAC_TELEOP_TASK> --xr --headless --device cpu
# wait for "IsaacTeleop teleoperation started"; the script AUTO-launches CloudXR (49100/48322)
```
Open the client on the Quest (push over adb to skip typing):
```bash
adb shell am start -a android.intent.action.VIEW \
  -d "https://nvidia.github.io/IsaacTeleop/client/release-1.3.x/#/sim/isaacsim"
```
On the Quest client: accept cert at `https://<PC_IP>:48322/` → set Server IP `<PC_IP>`, Port `48322`, **VR Immersive**, **Per-Eye 1024×896**, Codec **H.264** → **Connect**.

### Hard-won gotchas (these are about the runtime/streaming, so they carry over to any arm)
- **No window?** Isaac Lab 3.0 defaults to **headless**. Use `--visualizer kit` for a GUI window; use `--xr --headless` for windowless streaming that **auto-starts** the XR session (XR auto-start requires **both** `--xr` and `--headless`: `app_launcher.py` sets `auto_start = headless and xr`).
- **8 GB VRAM OOM** (`VK_ERROR_OUT_OF_DEVICE_MEMORY`, "worked a split second then white") → `--device cpu` + **low client Per-Eye resolution** + `--headless` (no extra desktop render) + RTX Real-Time (not Path Tracing). Don't gut lighting in the env cfg (made the robot render flat white); control VRAM via resolution instead.
- **Quest black but PC browser works** = **resolution mismatch**: log shows `ERROR Ignoring streaming dimensions 2048x2016, expected 1024x896`. Set the Quest's Per-Eye to **exactly** the server size. Width must be ×16, **height must be ×64** (896 ✓, 720 ✗).
- **Only ONE client streams at a time** — disconnect the PC browser before connecting the Quest.
- **`Port 49100 already in use`** crash = a stale CloudXR runtime survived a previous run; `pkill -9 -f cloudxr.runtime` (and kill the PID holding 49100) before re-running.
- `XR_ERROR_FEATURE_UNSUPPORTED xrCreateHandTrackerEXT` spam = harmless (Quest controllers, not hands).
- Teleop **forces AR profile** (`session_lifecycle.py` sets `/xr/profile/ar/enabled=True`) → no virtual environment backdrop, you see the robot over passthrough. (Experience type VR vs AR must also match the client mode.)
- Full write-up of the above lives at `~/robotics/QUEST_TELEOP_SETUP.md` on the Isaac Lab machine.

### What an Isaac Lab teleop task needs
The teleop script (`scripts/environments/teleoperation/teleop_se3_agent.py`) auto-detects
`env_cfg.isaac_teleop` (an `IsaacTeleopCfg` from the `isaaclab_teleop` extension). That config
provides the retargeting pipeline mapping Quest controller/headset pose → an end-effector pose
target, plus an XR anchor (where the user spawns relative to the scene). The EE pose target is
then consumed by the IK action (Part B).

---

## PART B — How Isaac Lab wires Pink IK (to replicate for our arm)

**Pink** = weighted-QP **differential** IK (https://github.com/stephane-caron/pink). Each *task*
defines a residual `e(q)` driven to zero; the solver (`daqp`, install `pin pin-pink==3.1.0 daqp==0.8.5`)
finds joint velocities minimizing the weighted task errors subject to joint limits; `v·dt` =
joint-angle deltas added to current joints → **position targets**. On solver failure it returns the
current joint positions (no jump).

Code: `~/robotics/IsaacLab-3.0/source/isaaclab/isaaclab/controllers/pink_ik/` + action term
`.../envs/mdp/actions/pink_task_space_actions.py`.

### Pieces
- **`PinkIKController`** (`pink_ik.py`): loads URDF (or converts USD→URDF at runtime), builds the
  kinematics config, instantiates tasks, builds **Isaac↔Pinocchio joint-ordering maps**.
  `compute(curr_joint_pos, dt)` runs `solve_ik(...)` and returns target joint positions.
- **`PinkIKControllerCfg`** (`pink_ik_cfg.py`): `usd_path`/`urdf_path`/`mesh_path`,
  `variable_input_tasks` (targets driven by the action each step), `fixed_input_tasks`,
  `num_hand_joints`, `base_link_name`, `articulation_name`, `fail_on_joint_limit_violation`.
  `joint_names`/`all_joint_names` are **auto-populated by the action term** — leave None.
- **Task configs** (`pink_task_cfg.py`): `FrameTaskCfg` (world-frame EE pose), `LocalFrameTaskCfg`
  (pose relative to a `base_link_frame_name`), `NullSpacePostureTaskCfg` (posture regularization for
  redundant arms), `DampingTaskCfg`. Fields: `frame` (URDF link), `position_cost` (cost/m),
  `orientation_cost` (cost/rad), `gain` (~0.075–1.0), `lm_damping` (e.g. 75, smooths near singularities).
- **Task impls** (`pink_tasks.py`): wrap `pink.tasks`; `LocalFrameTask` overrides error/Jacobian to a
  chosen base frame.
- **`PinkKinematicsConfiguration`** (`pink_kinematics_configuration.py`): keeps a **controlled/reduced**
  model (only the IK'd joints) AND a **full** model (frame poses can depend on non-controlled joints,
  e.g. a moving base).
- **Action term `PinkInverseKinematicsAction`** (`pink_task_space_actions.py`): one controller per env.
  **`action_dim = (#FrameTasks)×7 + num_hand_joints`** (7 = xyz + quat xyzw). Splits the action into
  per-task EE poses + hand joints, transforms target poses into the base-link frame, calls
  `controller.compute(...)`, applies joint position targets (+ optional gravity comp — **must be False
  on the Newton backend**).
  - **`PinkInverseKinematicsActionCfg`**: `pink_controlled_joint_names` (regex over USD joints),
    `hand_joint_names`, `target_eef_link_names` (dict task→URDF link), `controller=PinkIKControllerCfg`.

### Gotchas
- **USD (Isaac) joint order ≠ URDF (Pinocchio) order**, and **link/joint NAMES differ** between USD and
  the generated URDF. Task `frame`/`target_eef_link_names` use **URDF** names.
- If only `usd_path` is given, it auto-converts USD→URDF at runtime (needs Isaac Sim).

### Recipe to set up Pink IK for our arm (minimal single-arm)
1. Get our arm's **URDF** (or set `usd_path`). Note URDF link/joint names.
2. `PinkIKControllerCfg(usd_path/urdf_path=…, base_link_name=…, num_hand_joints=0,
   variable_input_tasks=[FrameTaskCfg(frame=<EE URDF link>, position_cost=…, orientation_cost=…,
   gain=…, lm_damping=…)])` (+ a `NullSpacePostureTaskCfg` if the arm is redundant).
3. `PinkInverseKinematicsActionCfg(pink_controlled_joint_names=[<arm joint regex>], hand_joint_names=[],
   target_eef_link_names={"ee": <EE link>}, controller=<above>)`.
4. Add to `env_cfg.actions`; register the task; for VR teleop also expose `env_cfg.isaac_teleop`
   (Part A) so the headset pose feeds the FrameTask target.
5. Action per env = `[x,y,z, qx,qy,qz,qw]` per controlled frame (+ hand joints). Tune costs/gain.
6. Reference example: `~/robotics/IsaacLab-3.0/source/isaaclab_tasks/.../locomanipulation/pick_place/configs/pink_controller_cfg.py` (G1 bimanual).

---

## PART C — THE REFERENCE CONFIG TO COPY: `Isaac-PickPlace-GR1T2-Abs-v0`

This is the **exact pattern to model our setup on** — a **bimanual humanoid, hand-tracking → Pink IK**, smooth (no DiffIK jitter). File:
`~/robotics/IsaacLab-3.0/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/pick_place/pickplace_gr1t2_env_cfg.py`.
Run it: `./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py --task Isaac-PickPlace-GR1T2-Abs-v0 --xr --headless --device cpu` (no `--enable_pinocchio` needed in beta2).

### End-to-end data flow (the mental model)
```
Quest hand tracking
  → IsaacTeleopDevice (supplies world_T_anchor transform + OpenXR handles)
  → retargeting pipeline (_build_gr1t2_pickplace_pipeline):
        HandsSource → transformed_hands (to sim world frame)
          → Se3AbsRetargeter ×2  (LEFT/RIGHT wrist → 7D EE pose [xyz + quat xyzw])
          → DexHandRetargeter ×2 (LEFT/RIGHT fingers → 11 joint angles each)
        → TensorReorderer → ONE flat action: [left_wrist(7), right_wrist(7), hand_joints(22)] = 36D
  → PinkInverseKinematicsAction.process_actions():
        splits 2×7 EE poses → transforms into base-link frame → sets each FrameTask target;
        22 hand joints → passthrough
  → .apply_actions(): PinkIKController.compute() solves the QP (FrameTasks + Damping + NullSpacePosture)
        → ARM joint targets;  hand joints appended  → set_joint_position_target
```

### The file's 3 pieces we replicate

**1. Retargeting pipeline** `_build_gr1t2_pickplace_pipeline()` (returns `(pipeline, [tuneable_retargeters])`):
- `hands = HandsSource(...)`; `transform_input = ValueInput("world_T_anchor", TransformMatrix())`; `transformed_hands = hands.transformed(...)`.
- **Per arm**: `Se3AbsRetargeter(Se3RetargeterConfig(input_device=HandsSource.LEFT/RIGHT, use_wrist_rotation=True, use_wrist_position=True, target_offset_yaw=…))` → connect `{HandsSource.X: transformed_hands.output(HandsSource.X)}`. (NOTE: GR1T2 uses **wrist pose** `use_wrist_*=True`; the Franka demo used the pinch midpoint `=False` — wrist is steadier. Right hand has `target_offset_yaw=180` because the USD control frame is rotated 180° vs OpenXR.)
- **Per hand**: `DexHandRetargeter(DexHandRetargeterConfig(hand_retargeting_config=<DexPilot YAML>, hand_urdf=<per-hand URDF>, hand_joint_names=[…11…], hand_side, handtracking_to_baselink_frame_transform=operator2mano))` → maps hand-tracking → finger joint angles. `operator2mano = (0,-1,0,-1,0,0,0,0,-1)`.
- **`TensorReorderer`** flattens everything into the action order the action term expects: `left_ee(7) + right_ee(7) + 22 hand-joint scalars` — **the hand-joint order in the reorderer MUST match `hand_joint_names` in the action cfg exactly.**

**2. `ActionsCfg.upper_body_ik = PinkInverseKinematicsActionCfg(...)`** (the core):
- `pink_controlled_joint_names`: the **14 arm joints** (7/arm) → these are the ones Pink IK solves.
- `hand_joint_names`: the **22 finger joints** → passthrough (NOT IK'd), exact order matching the reorderer.
- `target_eef_link_names={"left_wrist": "left_hand_pitch_link", "right_wrist": "right_hand_pitch_link"}` (URDF link names).
- `controller=PinkIKControllerCfg(base_link_name="base_link", num_hand_joints=22, fail_on_joint_limit_violation=False, variable_input_tasks=[ ... ])` where tasks are:
  - **`FrameTaskCfg` ×2** (one per wrist EE): `frame=<URDF wrist link>`, `position_cost=8.0`, `orientation_cost=1.0`, **`lm_damping=12`**, `gain=0.5` ← tracks the retargeter's wrist pose target.
  - **`DampingTaskCfg(cost=0.5)`** ← penalizes joint velocity (smoothness).
  - **`NullSpacePostureTaskCfg(cost=0.5, lm_damping=1, controlled_frames=[…wrists…], controlled_joints=[…arms + waist…])`** ← keeps redundant joints in a sane posture.
- **`action_dim = (#FrameTasks=2) × 7 + num_hand_joints(22) = 36`.**
- **This is why it's smooth** vs the Franka DiffIK: `lm_damping` on the frame tasks + the `DampingTask` + the `NullSpacePostureTask` all regularize the QP. DiffIK (`dls`) had none of that → jitter.

**3. `__post_init__`** wiring:
- `controller.usd_path = self.scene.robot.spawn.usd_path` + `urdf_output_dir = tmp` → **USD→URDF auto-converts at runtime** (Pink/Pinocchio needs a URDF; its link/joint names differ from USD — that's why `frame`/`target_eef_link_names` use URDF names).
- `self.xr = XrCfg(anchor_pos, anchor_rot)` → where the operator spawns relative to the scene.
- `self.isaac_teleop = IsaacTeleopCfg(pipeline_builder=lambda: _build_..._pipeline()[0], sim_device=…, xr_cfg=self.xr)` → connects pipeline ↔ teleop device.

### Recipe to clone this for OUR arm(s)
1. **Scene**: swap in our robot `ArticulationCfg` + our scene (table/objects/lights) in the scene cfg.
2. **Pipeline**: per arm `Se3AbsRetargeter` (HandsSource→wrist EE); per hand `DexHandRetargeter` with **our** DexPilot YAML + per-hand URDF + our finger `hand_joint_names` + `operator2mano`; `TensorReorderer` in our action layout.
3. **Action cfg**: our arm joints → `pink_controlled_joint_names`; our finger joints → `hand_joint_names` (same order as reorderer); `target_eef_link_names` = our URDF EE links; `controller` with a `FrameTaskCfg` per EE + `DampingTaskCfg` + `NullSpacePostureTaskCfg`. Tune `position_cost`/`orientation_cost`/`lm_damping`/`gain`.
4. **`__post_init__`**: set `controller.usd_path`, `xr`, `isaac_teleop`. Register the task id.
5. **21-DOF note**: if our "arm" is a 21-DOF arm+hand, decide which joints Pink IK solves (the arm/wrist DOFs → `pink_controlled_joint_names` + one `FrameTaskCfg` per EE) vs which are direct finger passthrough (`hand_joint_names` via a `DexHandRetargeter`). Pink IK handles the redundancy of a high-DOF arm via the `NullSpacePostureTask`.

---

## Open questions to resolve once repo specifics are provided
- Which arm/robot, and do we have its USD and/or URDF (+ meshes)?
- Where teleop/sim integration should live in this repo, and how it relates to the existing
  `autonomy/teleop` + `autonomy/simulation` work (keyboard teleop, IK already in progress).
- Whether to run Isaac Lab from `~/robotics/IsaacLab-3.0` or vendor a setup into this repo's docker.
- Bimanual vs single-arm (affects #FrameTasks and action_dim).
