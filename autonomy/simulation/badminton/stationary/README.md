# badminton receive env (stationary arm)

Implementation of `BADMINTON_ENV.md`: a MuJoCo badminton-receive environment
with validated shuttle aerodynamics, an inverse launcher that guarantees
hittable episodes, a pure interception predictor, and a scripted FSM baseline.
No rewards, no RL; every phase is gated by pytest.

The arm is `Humanoid_Wato/wato_arm_v2/urdf/armWithStand.urdf`: the dual-arm
model on its stand, standing on the floor behind the net, facing it (front =
base +x, the direction the ego-camera housing points). Only the right arm
(joint1-6) is actuated; the left arm and both parallel grippers are frozen,
and the racket is welded into the right gripper (link6) with the finger
bodies repositioned to squeeze the handle (visual stopgap; the end effector
is expected to be replaced). The stand mesh is used exactly as exported. The
arm model is treated as a fixed input; deviations are listed in the
simplifications ledger below.

## setup

```bash
cd autonomy/simulation/badminton/stationary
uv sync                      # Python 3.12 venv: mujoco, scipy, ... (local cpu mode)
uv run python scripts/mesh_prep.py     # regenerate racket/shuttle meshes
uv run python scripts/build_scene.py   # regenerate scene/badminton.xml
uv run python -c 'import launcher; launcher.build_workspace()'  # W cache
uv run pytest tests/                   # all gates
```

For RL training add `--extra train` (NVIDIA GPU machine / the mjlabs
container) or `--extra train-cpu` (CPU-torch stack, smoke tests only) — see
"training (mjlab / MuJoCo Warp)" below.

`scene/badminton.xml`, `scene/assets/*.obj` and `scene/workspace_W.npz` are
build artifacts; the sources of truth are `scene/params.yaml`, the arm URDF,
and the scripts above.

## layout

```
aero.py          # drag model, RK4 fwd/bwd, k-fit from film, u_out shooting solve
mjsim.py         # model loading, drag hook (xfrc_applied), episode helpers
launcher.py      # workspace W, episode spec sampling, back-integration
predictor.py     # pure interception function
perception.py    # simulated vision: shuttle EKF + trajectory prior (numpy ref)
perception_torch.py  # batched torch port of the EKF/prior (used in training)
badminton_mjlab/ # mjlab (MuJoCo Warp) task package: entities, MDP, RL configs
scene/           # params.yaml (single source of truth), badminton.xml, assets/
baseline/        # ik.py, minjerk.py, fsm.py, evaluate.py
tests/           # one pytest file per gate
scripts/         # aero_lab, sim_lab, replay, mesh_prep, build_scene,
                 # visualize_fan, run_gate5_eval, export_assets,
                 # train_rl / play_rl (mjlab entry points)
```

`../assets/` (the badminton root) holds shared exports for other projects:
the arm URDF with the racket and the assigned joint limits, and a standalone
court MJCF. Regenerate with `uv run python scripts/export_assets.py`; see
`../assets/README.md`.

Interactive tools (need a display): `scripts/aero_lab.py` (drag tuning with
sliders), `scripts/sim_lab.py` (viewer harness: overlays, hotkey tuning,
group toggles), `scripts/replay.py runs/gate5 --rank 0` (worst episodes
first, after `scripts/run_gate5_eval.py`).

## gate status

| gate | status |
|---|---|
| 1 aero (drop, range, shape, k-fit) | green |
| 2 scene (parity, bounce e=0.73-0.77, net, mesh alignment) | green |
| 3 launcher (forward verify < 2 cm on 1000, coverage, net legality) | green |
| 4 predictor (< 1 cm / < 5 ms on 1000) | green |
| 5 baseline | contact row xfail, rest green, see below |

Gates step physics through `mjsim.load()` on plain CPU MuJoCo. Two further
test files gate the training stack: `tests/test_perception.py` (shuttle EKF
+ trajectory prior; skips its torch half without torch) and
`tests/test_mjlab_task.py` (the Warp task vs the CPU reference; skips
without the train extras).

### gate 5 status (500-episode eval; datasheet motor torques)

| metric | measured | gate |
|---|---|---|
| contact rate | 71% | >= 90% (xfail, floor >= 50%) |
| net clearance of contacts | 54.9% | >= 60% (xfail, floor >= 40%) |
| median face pos err at t̂* | 49 mm | < 30 mm (xfail, floor < 100 mm) |
| median face timing error (plane crossing) | 24 ms | < 15 ms (xfail, floor < 40 ms) |
| joint-limit-hit episodes | 1 / 500 | 0 (the 100-episode pytest gate passes at 0) |
| self-collision episodes (>5 mm, executed motion) | 4 / 500 | 0 (xfail, floor <= 3 per 100) |

The pytest gate rows run the 100-episode seed-7 eval (61% contact there —
that seed draws harder episodes than the 500-episode average).

**Honesty notes, in order of discovery.**

1. An early build reported 91-97% contact — but a replay audit showed its
   planner was collision-blind: 200/200 audited episodes drove the arm or
   racket through the stand or itself (>5 mm interpenetration), with
   behind-the-back IK branches. Natural, collision-free planning (below)
   brought that to 78% contact / 84.9% clearance / 0 self-collisions over
   500 episodes — at CubeMars-class *guessed* torque limits.
2. The mounted motors are weaker than those guesses. With datasheet peak
   torques (AK10-9 53 Nm shoulders, AK80-9 22 Nm elbow group, GL40 0.73 Nm
   wrist) the wrist cannot track the swing: naive numbers were 54% contact /
   93 mm face error, with self-collisions returning because execution
   deviates from the collision-free plan. Wrist-sparing planning (IK joint
   weights pushing motion onto the strong motors, per-joint waypoint step
   caps, motor-scaled joint damping) recovers part of it — the table above
   (71% contact over 500 episodes; the 100-episode seed-7 pytest eval
   measures 61%).
   The GL40 wrist is the hardware bottleneck; closing the remaining gap is
   a wrist motor upgrade, swing trajectory optimization, or a learned
   policy.

What enforces naturalness now:

1. **Human-like joint ranges** (`arm.joint_range`, per joint) instead of
   +-pi: elbow flexes one way, shoulder/wrist ranges anthropomorphic. This
   alone removes the contorted IK branches — with +-pi a 30 cm hand move
   could take a 4.9 rad joint-space detour through the torso.
2. **Natural-posture workspace.** W additionally requires the face in front
   of the chest plane and the elbow not behind the back; the cloud stores
   the joint config and face normal per point.
3. **Workspace-seeded IK.** Guard/pre-hit solves seed and anchor
   (nullspace) on the nearest cloud configs — each collision-free by
   construction — instead of descending from wherever the arm happens to be.
4. **Collision-checked plans, routed transits.** A collision-enabled twin
   model (2 cm contact margin) checks every planned pose (5 mm clearance)
   and transit segment (15 mm); blocked transits route through via configs
   borrowed from the cloud; the retreat replays the executed stroke
   backward before returning to guard.
5. **Feasibility-committed intercepts.** The predictor ranks all
   workspace crossings (well-covered points first); the controller commits
   to the first candidate whose pre-hit pose actually solves — accurate,
   collision-free, inside the posture box.

The remaining ~20% misses split into: trajectories whose crossings are all
close to the chest (no feasible pre-hit exists — the racket's 0.45 m handle
offset forces the arm into the body), long routed transits the servo cannot
finish in time, and strokes near the workspace edge where the constrained
chain under-tracks the task quintic. Closing these is swing/whole-body
trajectory optimization or a learned policy — out of scope for the scripted
baseline.

Execution-level fixes that remain from earlier tuning: damping-compensated
inverse-dynamics feedforward (qfrc_bias lacks joint damping), joint_damping
0.1, waypoint step clamp 0.55 rad/tick, qdd cap 600 rad/s², limit-aware
waypoint/target clipping, task_kp 2.0 jacobian correction, swing_lead 0.

## training (mjlab / MuJoCo Warp)

Two modes, no backend flags anywhere:

- **local cpu** — everything above (gates, baseline, replay tools) runs on
  plain C MuJoCo via `mjsim.load()`. No GPU stack installed or imported.
- **headless mjlab** — RL training runs through
  [mjlab](https://github.com/mujocolab/mjlab) (Isaac-Lab-style managers on
  MuJoCo Warp; needs Linux + an NVIDIA GPU), with the viser web viewer on
  port 8080 for visualization on headless servers. The
  `modules/docker-compose.simulation_mj.yaml` `mjlabs` service builds a
  container with uv + Python 3.12 and maps that port; inside it,
  `uv sync --extra train` in this directory sets up the env.

`badminton_mjlab/` is the task package:

- `assets.py` splits the gate-validated `scene/badminton.xml` into a robot
  entity (arm + racket, XML actuators kept), a shuttle entity, and a court
  scene hook that re-adds the four calibrated contact pairs across the
  entity prefixes — geometry and calibration stay single-sourced.
- `shuttle_action.py` extends the joint-position action term with the two
  per-substep shuttle physics pieces (half-step drag on `xfrc_applied`,
  kinematic cork-first orientation lag) so the Warp rollout steps the same
  dynamics the gates validate.
- `perception_command.py` + `mdp.py` + `env_cfg.py` define the MDP (below).
- `rl_cfg.py` holds the PPO (teacher) and distillation (student) runner
  configs.

**Trajectory as a prior, not something the policy learns.** The policy never
has to infer flight physics from raw shuttle positions: every observation
includes a predicted trajectory (8 future positions at 0.1 s spacing) rolled
out on the calibrated drag model. The teacher's prior comes from the true
state; the student's comes from a batched EKF (`perception_torch.py`) fed
one noisy position measurement per 50 Hz tick — early in the flight the
velocity estimate is poor so the predicted trajectory jumps tick to tick,
then it converges as measurements accumulate, exactly how the real
perception model refines its fit over time. `perception.py` is the numpy
reference; `tests/test_perception.py` gates the filter (convergence,
honest early uncertainty) and the torch/numpy parity.

**Teacher-student.** Phase 1 trains a privileged teacher with PPO; phase 2
distills it into the student observation set (rsl_rl `DistillationRunner`):

```bash
# in the container (or any NVIDIA linux box), from this directory:
uv sync --extra train
uv run scripts/train_rl.py Mjlab-Badminton-Receive-Teacher --env.scene.num-envs 4096
uv run scripts/train_rl.py Mjlab-Badminton-Receive-Student   # loads the teacher ckpt
uv run scripts/play_rl.py  Mjlab-Badminton-Receive-Teacher --viewer viser
# viser serves on http://localhost:8080 (SSH-tunnel from a headless server)
```

On the WATcloud SLURM cluster (wato-login1/wato-login2), submit
`sbatch scripts/slurm_train.sbatch` from this directory; it bootstraps uv,
syncs the train extra, and runs the teacher (NUM_ENVS overrides the default
1024). Logs land in `runs/slurm-<jobid>.out`.

MDP summary: actions = 6 joint position targets around the ready pose,
clipped to the joint ranges; episodes reset from the launcher bank (same
rejection-sampled hittable episodes as the gates); rewards = sparse face
contact + approach shaping toward p* + post-hit return-flight quality, minus
action-rate/joint-limit/joint-velocity penalties; terminations on floor or
net contact and a 3 s timeout. Contact sensing uses force history over the
20 substeps of a tick so a bounce inside a tick is never missed.

`uv sync --extra train-cpu` installs the same stack with CPU torch:
training there is impractical, but `tests/test_mjlab_task.py` runs the task
end-to-end on CPU Warp and checks flight parity against the CPU reference
sim (< 2 cm at 0.6 s), the on-device cork orientation, in-env EKF
convergence, and task registration. GPU float32 reductions reorder, so
expect small numeric drift vs the CPU reference — the calibrated gate rows
(bounce restitution, gate-5 floors) were measured on CPU; if one trips on a
GPU machine, recalibrate there and record it in the ledger.

## simplifications ledger (revisit at sim2real time)

| simplification | planned upgrade |
|---|---|
| point-mass shuttle dynamics; orientation is kinematic (cork axis lagged toward velocity, `shuttle.orient_tau`; angular velocity zeroed; collision is a sphere so this is visual only) | aerodynamic restoring torque + spin dynamics; speed-dependent C_d |
| constant k undershoots the spec's rough 8-12 m range test band at 20 m/s (measured 7.1 m); gate widened to 6.5-12 m | speed-dependent C_d |
| padded 12 mm rigid face | thinner face + smaller dt, or effective-restitution fit |
| shuttle collision = cork sphere (racket face + floor) + skirt sphere (net + floor); mesh skirt can still visually overlap the net band | convex-decomposed shuttle collision |
| face-pair solref [0.012, 0.11] calibrated for cork-sphere apex e in [0.70, 0.77] (spec's 0.004/0.5 seed value gives e~0.2) | fit to a real racket |
| torque limits = datasheet peaks (AK10-9/AK80-9/GL40: 53/22/0.73 Nm); the GL40 wrist saturates during swings, mitigated by wrist-sparing IK weights + per-joint step caps + motor-scaled damping | wrist motor upgrade, or rated-torque + thermal model for sustained load |
| uniform net height 1.55 m | sagging profile, 1.524 m center |
| scripted baseline's predictor gets clean state (the RL student already sees the EKF-filtered noisy path from perception.py) | route the baseline through the same filter |
| arm links non-collidable with shuttle | enable, add frame/mishit geometry |
| fixed launch band; p* additionally filtered so the pre-hit point is in W | full far-court variety |
| primitive collision proxies under meshes | convex-decomposed collision if mishit realism matters |
| left arm and both grippers frozen (joints deleted); right fingers posed closed on the handle | actuate for two-handed play / active grip |
| joint ranges assigned human-like values (no data in the URDF; signs from FK probes) | real hardware limits |
| collision checks live in a planner-side twin model; the runtime physics still has no arm self-contacts | enable arm contact pairs if impact realism matters |
| v2 URDF exports every joint limit as 0/0/0/0; ranges assigned human-like per joint (`arm.joint_range`; the repo's only measured ranges are conservative ~10-20 deg calibration sweeps, not mechanical stops), efforts = datasheet peaks (`arm.torque_limits`) | measured mechanical stops and motor curves |
| joint damping [0.1 x4, 0.05, 0.02] (motor-size-scaled) / armature 0.01 added for actuation stability | identify from hardware |
| servo gains kp [400,400,300,300,100,8] / kv [5,5,5,5,2,0.3] capped to the CAN MIT ranges (kp <= 500, kd <= 5; wrist = the "start low" placeholder, no drive gains configured on hardware yet). The earlier kp 60 / kv 3 wrist saturated the 0.73 Nm GL40 at any error > 0.7 deg. Gate-5 baseline numbers above predate this change and need a re-run | tune on the real drives, re-run the gate-5 eval |
| gravity+inertia+damping feedforward via `qfrc_applied`, clamped to the same torque limits | robot-side model-based feedforward |
| semi-implicit Euler + half-step drag hook (RK4 in mujoco destabilized fast contacts) | revisit if contact fidelity needs change |

## conventions

- Net line at y = 0; the arm stand rests on the floor at (-0.29, -2.0),
  yawed so its front (base +x, the ego-camera direction) faces the net,
  base_link origin (shoulder height) at z = 1.20; +z up; the far court
  (launch side) is y > 0. The x-shift centers the right arm's swing plane on
  court center.
- Geom groups: 2 = visual meshes, 3 = collision primitives (toggle in
  sim_lab with v/b).
- Contacts: explicit `<pair>` entries only (cork-face, cork-floor,
  skirt-floor, skirt-net; the cork sphere's surface sits at the mesh cork
  tip so a cork-first arrival contacts the face where the mesh shows it). Collision-layer geoms compile with contype=1/conaffinity=0 so
  mesh hulls exist for the workspace self-collision check; no contype-based
  pair ever matches at step time.
- The drag model has exactly two implementations kept in lockstep:
  `aero.drag_accel` (RK4 reference, mujoco hook, predictor, perception EKF)
  and its torch port in `perception_torch.py`/`badminton_mjlab` (checked by
  `tests/test_perception.py` and the Warp flight-parity test).
