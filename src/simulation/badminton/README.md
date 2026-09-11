# Badminton receive — stationary arm, RL

MuJoCo simulation of the WATonomous arm receiving badminton serves, and the
RL pipeline that trains the receive policy (mjlab / MuJoCo Warp, rsl_rl PPO).

## Setup

```bash
uv sync --extra train          # Linux + NVIDIA GPU (CPU: --extra train-cpu, smoke tests only)
uv run scripts/build_scene.py  # regenerate scene/badminton.xml after editing scene/params.yaml
uv run pytest -q
```

The arm is `assets/pioneer_bimanual_arm/urdf/pioneer_bimanual_arm_stand.urdf`
(repo root). `scene/params.yaml` is the single source of truth (shuttle aerodynamics,
arm joint ranges / torque and speed limits, control gains, launcher bank,
perception noise). Note the speed cap: `control.target_velocity_max` is in
the hardware `joint_command` units, where the effective steady-state speed
is `(1 - low_pass_alpha) * velocity_max` (15%).

## Train

```bash
# 1. teacher: PPO on privileged state (true shuttle state + intercept point)
uv run scripts/train_rl.py Mjlab-Badminton-Receive-Teacher --env.scene.num-envs 1024

# 2. student: distil the teacher into a policy on realistic perception
#    (EKF-tracked shuttle + trajectory prior). Symlink the teacher run dir
#    into logs/rsl_rl/badminton_student/ (any name containing "badminton_teacher").
uv run scripts/train_rl.py Mjlab-Badminton-Receive-Student

# 3. fine-tune the student with PPO on its own reward (recovers what
#    distillation loses to perception noise; critic sees privileged state)
uv run scripts/student_to_ppo.py --student logs/rsl_rl/badminton_student/<run>/model_1499.pt \
    --out logs/rsl_rl/badminton_student_ppo/init/model_0.pt
uv run scripts/train_rl.py Mjlab-Badminton-Receive-Student-PPO \
    --agent.resume True --agent.load-run init --agent.load-checkpoint model_0.pt
```

Runs log to Weights & Biases (project `mjlab`); checkpoints land in
`logs/rsl_rl/<experiment>/<timestamp>/`. rsl_rl opens a new timestamped
directory on every launch, including resumes.

## Evaluate / watch

```bash
uv run scripts/eval_rl.py --task Mjlab-Badminton-Receive-Student-PPO \
    --checkpoint-file <model.pt> --episodes 8192      # hit rate, return quality, joint feasibility
uv run scripts/play_rl.py Mjlab-Badminton-Receive-Student-PPO --viewer viser \
    --checkpoint-file <model.pt>                      # http://localhost:8080
```

## Layout

```
aero.py, launcher.py, perception*.py, predictor.py, mjsim.py   physics, serve bank, EKF prior, plain-MuJoCo sim
baseline/        scripted receive (IK + min-jerk swing) used as the reference before RL
humanoid_badminton/ the mjlab task: env config, rewards, observations, action moderation, perception command
scripts/         build_scene, mesh_prep, train_rl, student_to_ppo, eval_rl, play_rl
scene/           params.yaml, generated badminton.xml, meshes
tests/           gates 1-5 (aero, scene, launcher, predictor, baseline) + perception + mjlab task
```

Policy I/O: 61 inputs at 50 Hz (joint pos/vel, racket face pose, last
action, EKF shuttle state + 8-point trajectory prior, EKF uncertainty) →
6 joint-position targets, rate-limited and low-passed like the hardware
path; after the hit the arm returns to a rest pose.
