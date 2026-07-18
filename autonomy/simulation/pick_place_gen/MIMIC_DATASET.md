# Isaac Lab Mimic pick-and-place dataset (`scaled_big.hdf5`)

State-only IL dataset for the **wato_bimanual_arm (left arm)** pick-and-place task:
20 cuRobo-planned seed demos (`record_mimic_source_demos.py`) → grasp-subtask
annotation (`annotate_demos.py`) → scaled to 530 by **Isaac Lab Mimic**
(`generate_dataset.py`). It's the *input* to training — no policy has been trained.

| | |
|---|---|
| Path | `autonomy/simulation/Humanoid_Wato/HumanoidRL/datasets/scaled_big.hdf5` (~147 MiB, **gitignored**) |
| Demos | **530**, all `success=True`, 375,474 samples |
| Episode length | 611–926 steps (mean 708) |
| Task | Pick the cube, place it on the `place_target` marker (**table** mode) |
| Env | `Isaac-PickPlace-BimanualLeft-Mimic-v0`, control **50 Hz** (`dt=0.01`, `decimation=2`) |
| Modality | **State only — no camera images** |

## Layout

Isaac Lab HDF5 (robomimic-style):

```
data                                  attrs: env_args (JSON), total
└── demo_<i>                          attrs: num_samples, success
    ├── actions                (T, 8)   commanded joint targets
    ├── processed_actions      (T, 8)
    ├── obs/
    │   ├── joint_pos          (T, 8)   left-arm joint positions
    │   ├── joint_vel          (T, 8)
    │   ├── object_pose        (T, 7)   pos xyz + quat wxyz, ROBOT ROOT frame
    │   └── actions            (T, 8)   previous action
    ├── states/                         full sim state, for replay / reset_to
    │   ├── articulation/robot/{joint_position (T,16), joint_velocity (T,16),
    │   │                       root_pose (T,7), root_velocity (T,6)}
    │   └── rigid_object/{object,place_target}/{root_pose (T,7), root_velocity (T,6)}
    └── initial_state/                  same tree, (1, ...) — for env.reset_to()
```

### Action / observation space (8-dim)

`actions` and `obs/joint_pos` are joint position targets / measurements, ordered
6 arm + 2 gripper:

| idx | joint | |
|---|---|---|
| 0–5 | `joint1L, joint2l, joint3l, joint4l, joint5l, joint6l` | left arm |
| 6 | `joint7l` | gripper, limit `[-0.05, 0.00]` |
| 7 | `joint8l` | gripper, limit `[0.00, 0.05]` |

Gripper (per `wato_constants.py`, the corrected convention):
**open `(0.0, 0.0)`**, **closed `(-0.05, +0.05)`**.

`states/.../joint_position` holds all **16** joints (both arms); the left-arm 8 are
indices `[1,3,5,7,9,11,14,15]` — but prefer `obs/joint_pos`, already the right 8.

## Loading

```python
import h5py, json, numpy as np

f = h5py.File("datasets/scaled_big.hdf5", "r")
json.loads(f["data"].attrs["env_args"])              # env name + sim args
keys = sorted(f["data"], key=lambda s: int(s.split("_")[1]))   # demo_0 … demo_529

d = f["data"][keys[0]]
actions   = np.array(d["actions"])                   # (T, 8) joint targets
joint_pos = np.array(d["obs/joint_pos"])             # (T, 8)
obj_pose  = np.array(d["obs/object_pose"])           # (T, 7) pos + quat(wxyz)
```

## Training

`robomimic` 0.4.0 is installed, and Isaac Lab ships
`scripts/imitation_learning/robomimic/{train.py, play.py, robust_eval.py}`, which
expect this layout. Build obs from `joint_pos`, `joint_vel`, `object_pose`
(+ `obs/actions` for a previous-action channel); regress `actions`.

```bash
cd /workspace/humanoid/autonomy/simulation/Humanoid_Wato/HumanoidRL
export PYTHONPATH=$(pwd)

$ISAACLAB/isaaclab.sh -p /workspace/isaaclab/scripts/imitation_learning/robomimic/train.py \
    --task Isaac-PickPlace-BimanualLeft-Mimic-v0 \
    --dataset datasets/scaled_big.hdf5 --algo bc
```

Not run end-to-end here — check `train.py --help`, the algo/config flags vary by
Isaac Lab release.

## Regenerating / extending

Seed + annotated intermediates were deleted, so more demos means re-running all
three stages (~10 min for stages 1–2).

```bash
cd /workspace/humanoid/autonomy/simulation/Humanoid_Wato/HumanoidRL
export PYTHONPATH=$(pwd)
export PYTHONUNBUFFERED=1        # Isaac Sim buffers stdout; without this you see no progress
MIMIC=/workspace/isaaclab/scripts/imitation_learning/isaaclab_mimic

# 1) cuRobo seed demos (~5 min for 20; table mode = no --task_params)
$ISAACLAB/isaaclab.sh -p ../../pick_place_gen/record_mimic_source_demos.py --headless \
    --num_episodes 20 --seed 0 --output_file datasets/source.hdf5

# 2) annotate the grasp subtask (~3 min)
$ISAACLAB/isaaclab.sh -p ../../pick_place_gen/run_isaaclab_mimic_script.py $MIMIC/annotate_demos.py \
    --task Isaac-PickPlace-BimanualLeft-Mimic-v0 --headless --auto \
    --input_file datasets/source.hdf5 --output_file datasets/annotated.hdf5

# 3) scale up (~10 attempts/min, ~20% yield; writes incrementally — Ctrl-C anytime)
$ISAACLAB/isaaclab.sh -p ../../pick_place_gen/run_isaaclab_mimic_script.py $MIMIC/generate_dataset.py \
    --task Isaac-PickPlace-BimanualLeft-Mimic-v0 --headless \
    --input_file datasets/annotated.hdf5 --output_file datasets/scaled_big.hdf5 \
    --generation_num_trials 2000
```

- **Never pass `--enable_cameras`** — the datagen env is state-only, and it spins
  up the RTX pipeline, which deadlocks during headless scene setup here.
- Stages 2–3 go via `run_isaaclab_mimic_script.py`, which registers this custom
  task with Isaac Lab's stock Mimic scripts and surfaces tracebacks before Kit's
  shutdown swallows them.
- Progress line: `N/M (X%) successful demos generated by mimic` (`N` saved, `M` attempted).
