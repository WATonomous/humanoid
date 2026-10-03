# humanoid-garment-fold

Bimanual **garment-folding** Isaac Lab task for the WATonomous `pioneer_bimanual_arm`,
ported from the LeHome Challenge. Fold long/short tops and long/short pants
on a table; success is judged on the cloth's particle geometry.

## Status

| Piece | State |
|---|---|
| `GarmentEnv` + particle cloth + success checker + reward | ✅ vendored (`lehome@a805ad2`, Apache-2.0, see `NOTICE`) |
| Retargeted to one `pioneer_bimanual_arm` | ✅ `GarmentPioneerEnv`, gym id `Humanoid-GarmentFold-Bimanual-Pioneer-v0` |
| Scene + garment on the table | ✅ loads, verified in this repo's `isaac_lab` image |
| Arm base pose vs. the garment | ✅ `(0, -0.40, 0.95)` -- reach-tested (`scripts/fk_reach_check.py`); re-sinks the stand ~25cm into the floor as a trade-off, see `garment_pioneer_cfg.py` |
| **Arm default joint pose** | ⚠️ droops at rest -- idle pose isn't fold-ready. Reach *while driven* works fine; this is only about the resting pose. |
| Wrist camera offsets | ✅ fixed -- CAD-sourced mount from `pioneer_humanoid.arm_params.CAMERAS` |
| Full 600-step episode / success-checker | ✅ runs clean end-to-end (~55s on an RTX 4060) |
| `keyboard_teleop.py --scene garment_fold` | ✅ works -- see "Interactive teleop" below |
| Teleop → demos → LeRobot training | ❌ recording/training pipeline not wired |

## Layout

```
garment_fold_task/
├── NOTICE                       what was vendored/modified, §4 of the repo's own Apache-2.0 LICENSE
├── pyproject.toml
├── vendor_assets/
│   ├── garments/Release/       1 sample garment per category (committed, ~6 MB)
│   ├── scenes/Table038/        fallback table (committed, ~1.2 MB)
│   └── scenes/marble/          Scene_00_Apartment.usd — .gitignore'd, you copy it
└── humanoid_garment_fold/
    ├── tasks/
    │   ├── garment_env.py            VENDORED — the GarmentEnv base
    │   ├── garment_env_cfg.py        VENDORED — base cfg (robot fields removed)
    │   ├── challenge_garment_loader.py  VENDORED
    │   ├── garment_pioneer_cfg.py    NEW — pioneer config
    │   ├── garment_pioneer_env.py    NEW — pioneer env (subclass)
    │   ├── teleop_scene.py           NEW — keyboard_teleop.py --scene registration
    │   └── __init__.py               gym.register(...)
    ├── assets/
    │   ├── garment_object.py         VENDORED — PhysX particle-cloth garment
    │   └── scene.py                  adapted — scene USD paths
    ├── utils/
    │   ├── success_checker_garment.py  VENDORED — fold-quality check
    │   └── logger.py                 NEW — console shim
    └── config/particle_garment_cfg.yaml  VENDORED — particle solver params
```

## The retarget

Upstream drives two SO101 follower arms (`left_arm`/`right_arm`, 12-dim action).
This drives **one `pioneer_bimanual_arm`** instead: left chain `joint1L..joint6l`
+ gripper, EE `link6l`; right chain `joint1..joint6` + gripper, EE `link6`.
Action stays 12-dim (`[left ×6, right ×6]` joint-position targets); grippers
held open. Names/cfg come from `pioneer_humanoid` (see `bimanual_arm.py`).

The pioneer arm's front axis is `+X`; the garment sits at world `~(0,0,0.63)`,
so the base is rotated +90° about Z. `_build_worksurface()` tries, in order:
optional NuRec backdrop, `Scene_00_Apartment.usd` (default — gitignored,
~19MB, copy it in), ground + vendored `Table038.usd`, ground only.

## Run it

Use the **`simulation_isaac_garment`** watod module, not `simulation_isaac` --
same published `isaac_lab` image plus two fixes (Isaac Lab 2.3.0 downgrade for
a `TiledCamera` freeze bug, `open3d` for the success checker) that aren't safe
to bake into the shared Dockerfile without broader testing. See
`docker/simulation/isaac_lab/isaac_lab_garment.Dockerfile` for exactly what
and why.

```bash
# Copy the whole Assets/scenes/marble/ folder from the LeHome challenge into
# vendor_assets/scenes/marble/ first -- the .usd references the .usdz next to
# it and won't load without both.
ACTIVE_MODULES="simulation_isaac_garment" ./watod up -d
./watod -t simulation_isaac_garment_dev
# inside the container:
pip install -e src/pioneer_humanoid src/simulation/garment_fold_task --no-deps --no-build-isolation
cd src/simulation/garment_fold_task
isaaclab.sh -p scripts/smoke_test.py --garment Top_Long_Seen_1       # builds the env, resets, writes 4 camera PNGs
isaaclab.sh -p scripts/full_episode_test.py --garment Top_Long_Seen_1 --steps 600  # full episode, no crash expected
```

Both verified against a real `docker build` of `isaac_lab_garment.Dockerfile`,
not just a hand-patched container.

### Interactive teleop

`src/teleop/keyboard_teleop/keyboard_teleop.py --scene garment_fold` drives
the real arm into this scene with a live keyboard -- useful for personally
checking gripper-to-garment contact. Needs a real display (not headless):

```bash
pip install -e src/pioneer_humanoid src/simulation/garment_fold_task src/simulation/isaac_scenes --no-deps --no-build-isolation
cd src/teleop/keyboard_teleop
GARMENT_NAME=Top_Long_Seen_1 isaaclab.sh -p keyboard_teleop.py --scene garment_fold --enable_cameras
```

The garment (a particle cloth, not a declarative `AssetBaseCfg`) is built by a
`post_init(scene, sim)` hook -- a small, generic addition to
`humanoid_isaac_scenes/_register.py` and `keyboard_teleop.py`, a no-op for
every scene that doesn't need one. `tasks/teleop_scene.py` is the single
source of truth; `scripts/scene_flag_check.py` verifies the whole path
headless, short of the live keyboard loop itself.

## To finish

1. **Reach works; grasping/pinching doesn't.** `scripts/reach_pose_ik.py`
   gets within ~1-5mm of the garment. The 4 gripper prismatic joints still
   need tuning to actually pinch fabric, and the *default/idle* pose droops
   (separate from reach, which works fine when actively driven).
2. **Full garment set**: `hf download lehome/asset_challenge` → `garment_cfg_base_path`.
3. **Data + training**: pioneer teleop → record demos → `lerobot-train` with
   LeHome's ACT / DP / SmolVLA configs.

## Notes from the port

* `Scene_00_Apartment.usd` composes only a table when opened minimally; the
  photoreal apartment (NuRec `.usdz`) renders through the full Isaac Sim
  pipeline. Both are in the LeHome `Assets/scenes/marble/`; `scene_v1.usd` (the
  room mesh) ships in neither the repo nor the HF dataset.
