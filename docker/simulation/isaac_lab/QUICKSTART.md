# Isaac Lab quickstart

Copy-paste path from a fresh clone to **training and playing a Pioneer RL policy** in sim.
Stack: Isaac Lab 2.3.2 / Sim 5.1 / RSL-RL / watod `simulation_isaac`.

Full reference: [README.md](README.md). Tasks and runners: [`src/simulation/README.md`](../../../src/simulation/README.md).

---

## 0. One-time (host)

```bash
cd ~/Desktop/humanoid

xhost +local:docker
mkdir -p ~/docker/isaac-sim/{cache/kit,cache/ov,cache/pip,cache/glcache,cache/computecache,logs,data}
```

Create `watod-config.local.sh` (git-ignored):

```bash
cat > watod-config.local.sh <<'EOF'
ACTIVE_MODULES="simulation_isaac"
MODE_OF_OPERATION="develop"
EOF
```

---

## 1. Build image (first time ~12 min; rebuild ~5–15 min)

```bash
cd ~/Desktop/humanoid
./watod build simulation_isaac_dev
```

If `import torch` fails inside container (packaging error), rebuild clean:

```bash
./watod build --no-cache simulation_isaac_dev
```

---

## 2. Start container (host)

```bash
./watod up -d
./watod -t simulation_isaac_dev
```

---

## 3. Sanity check (inside container)

```bash
$PYTHON -c "import torch; print(torch.__version__)"   # expect 2.7.0+cu128
$PYTHON -c "import lerobot; print('ok')"
```

Container env (from `.bashrc`): `$ISAACLAB`, `$HUMANOID_ROOT`, `$RL_RUNNERS` (RSL-RL train/play scripts), `$TASK_ROOT` (SO101 IL).

Aliases: `rl-train`, `rl-play`, `train-policy`, `eval-policy`, `record-demos`.

**Open Isaac Sim GUI** (no task script — plain simulator). Container runs as root, so allow that:

```bash
OMNI_KIT_ALLOW_ROOT=1 $ISAACLAB/isaaclab.sh -s
```

---

## 4. RL tasks — in-hand / locomotion / push / etc. (inside container)

Tasks live in `src/simulation/humanoid_rl_tasks/`, runners in `$RL_RUNNERS` — see
[src/simulation/README.md](../../../src/simulation/README.md). Repo is
bind-mounted; checkpoints under `$HUMANOID_ROOT/outputs/rl/` on host.

```bash
cd $HUMANOID_ROOT

# In-hand cube reorientation — train
rl-train --task=Isaac-Repose-Cube-PioneerHand-v0 --headless

# Play (GUI; omit --headless)
rl-play --task=Isaac-Repose-Cube-PioneerHand-Play-v0 --num_envs=1

# Locomotion — Pioneer humanoid V1 (flat)
rl-train --task=Isaac-Locomotion-Flat-PioneerHumanoid-v0 --headless
rl-play --task=Isaac-Locomotion-Flat-PioneerHumanoid-Play-v0 --num_envs=1
```

Task docs: `src/simulation/humanoid_rl_tasks/humanoid_rl_tasks/<task>/*.md`.

---

## 5. If Isaac hangs (host)

Ctrl+C often fails after an Isaac crash. From the host:

```bash
./watod down
# or: docker kill $(docker ps -q --filter name=simulation_isaac)
./watod up -d
```

---

## What runs in this container

| Workload | Where to look |
|----------|---------------|
| RL tasks (all) | §4 above; `humanoid_rl_tasks/` + runners in `$RL_RUNNERS` |
| Quest teleop | [`src/teleop/quest_teleop/README.md`](../../../src/teleop/quest_teleop/README.md) |
| SO101 imitation learning (ACT train, sim eval, record demos) | [`src/simulation/so101_vial_task/README.md`](../../../src/simulation/so101_vial_task/README.md) |
