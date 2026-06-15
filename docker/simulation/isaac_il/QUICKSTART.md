# SO101 sim IL — copy-paste commands

Exact workflow used for **HF dataset → ACT train → sim eval** (no physical arm).
Stack: Isaac Lab 2.3.2 / Sim 5.1 / LeRobot 0.4.3 / ACT / watod `simulation_il`.

Full reference: [README.md](README.md). Task/env details: [`autonomy/simulation/so101_vial_task/README.md`](../../../autonomy/simulation/so101_vial_task/README.md).

---

## 0. One-time (host)

```bash
cd ~/Desktop/humanoid

xhost +local:docker
./assets/lerobot/sync_so101_vial_assets.sh --full
mkdir -p ~/docker/isaac-sim/{cache/kit,cache/ov,cache/pip,cache/glcache,cache/computecache,logs,data}
```

Create `watod-config.local.sh` (git-ignored):

```bash
cat > watod-config.local.sh <<'EOF'
ACTIVE_MODULES="simulation_il"
MODE_OF_OPERATION="develop"
EOF
```

---

## 1. Build image (first time ~12 min; rebuild ~5–15 min)

```bash
cd ~/Desktop/humanoid
./watod build simulation_il_dev
```

If `import torch` fails inside container (packaging error), rebuild clean:

```bash
./watod build --no-cache simulation_il_dev
```

---

## 2. Start container (host)

```bash
./watod up -d
./watod -t simulation_il_dev
```

---

## 3. Sanity check (inside container)

```bash
$PYTHON -c "import torch; print(torch.__version__)"   # expect 2.7.0+cu128
$PYTHON -c "import lerobot; print('ok')"
```

Container aliases (from `.bashrc`): `$ISAACLAB`, `$TASK_ROOT`, `il-train`, `il-eval`, `il-record`.

---

## 4. Train ACT on public HF dataset (inside container)

Dataset: [CursedRock17/so101_teleop_vials_sim_and_real](https://huggingface.co/datasets/CursedRock17/so101_teleop_vials_sim_and_real) — **140 demos**, 26,516 frames.

**Smoke test** (10k steps ≈ 3 epochs over 140 demos, ~1–2 h):

```bash
mkdir -p /workspace/humanoid/outputs/train/so101_hf_act

il-train \
  --dataset.repo_id=CursedRock17/so101_teleop_vials_sim_and_real \
  --policy.type=act \
  --policy.push_to_hub=false \
  --output_dir=/workspace/humanoid/outputs/train/so101_hf_act \
  --policy.device=cuda \
  --steps=10000 \
  --batch_size=8 \
  --save_freq=5000 \
  --job_name=so101_hf_smoke
```

**Longer run** (better policy):

```bash
il-train \
  --dataset.repo_id=CursedRock17/so101_teleop_vials_sim_and_real \
  --policy.type=act \
  --policy.push_to_hub=false \
  --output_dir=/workspace/humanoid/outputs/train/so101_hf_act_50k \
  --policy.device=cuda \
  --steps=50000 \
  --batch_size=8 \
  --save_freq=10000 \
  --job_name=so101_hf_50k
```

Checkpoint path when done:

```
/workspace/humanoid/outputs/train/so101_hf_act/checkpoints/last/pretrained_model
```

Same path on host: `~/Desktop/humanoid/outputs/train/so101_hf_act/...`

### Train pitfalls (learned the hard way)

| Wrong | Right |
|-------|-------|
| `lerobot-train` (bare CLI) | `il-train` or `$PYTHON -m lerobot.scripts.lerobot_train` |
| `-m lerobot.scripts.train` | `-m lerobot.scripts.lerobot_train` |
| `--training.num_epochs=10` | `--steps=10000` |
| omit `--policy.push_to_hub=false` | always set `false` for local-only training |
| `pip install humanoid-il[sim]` in image | breaks Isaac torch — Dockerfile uses `--no-deps` only |

---

## 5. Eval ACT in sim (inside container)

**Must** run from `$TASK_ROOT`. **Do not** pass `--rename_map` for local ACT (cameras already `ego` / `external_D455`).

```bash
cd /workspace/humanoid/autonomy/simulation/so101_vial_task

PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p scripts/lerobot_eval.py \
  --task Lerobot-So101-Teleop-Vials-To-Rack-DR-Eval \
  --policy_type lerobot \
  --policy_path /workspace/humanoid/outputs/train/so101_hf_act/checkpoints/last/pretrained_model \
  --num_episodes 5
```

- Isaac GUI opens (X11 to host display).
- **R** = manual reset world.
- Prints success rate when episodes finish.
- 10k-step smoke policy can still succeed sometimes (e.g. 33–50% in early tests).

### Eval pitfalls

| Wrong | Right |
|-------|-------|
| Run from `/workspace/humanoid` | `cd $TASK_ROOT` first |
| `--rename_map '{"ego":"observation.images.ego",...}'` | omit for ACT — causes `KeyError: 'ego'` |
| Ctrl+C when frozen after crash | `./watod down` from host |

`[WARNING] No textures found` — run asset sync on host, then re-eval.

---

## 6. Stop (host)

```bash
exit                    # leave container shell
cd ~/Desktop/humanoid
./watod down
```

If Isaac hung and Ctrl+C fails:

```bash
./watod down
# or: docker kill $(docker ps -q --filter name=simulation_il)
./watod up -d
```

---

## 7. Optional — record your own demos (needs USB leader)

```bash
cd $TASK_ROOT
mkdir -p /workspace/humanoid/datasets/record_so101_gym/001

PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p scripts/lerobot_agent.py \
  --task Lerobot-So101-Teleop-Vials-To-Rack-DR \
  --port /dev/ttyACM0 \
  --repo_root /workspace/humanoid/datasets/record_so101_gym/001 \
  --save_mp4 --depth --instance_id_seg
```

---

## What stays on host (not docker)

| Workload | Where |
|----------|-------|
| HumanoidRL / in-hand RL | host `env_isaaclab` + `/home/hy/IsaacLab` |
| SO101 IL train + sim eval | `simulation_il` docker |
| Quest / Wato teleop | `simulation` docker (Sim 4.5) |
