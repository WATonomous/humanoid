# pick_place_gen — Archived: cuRobo scripted demo generator

The cuRobo-based scripted demonstration generator that used to live in this
directory (`generate_demos.py`, `orchestrator.py`, `curobo_expert.py`,
`build_wato_robot_cfg.py`, `validate_curobo_plan.py`, `curobo_cfg/`, and the
`isaac_lab_il_datagen` Docker module/container) has been removed. Seed demos
for this task are being replaced with VR-teleoperated demonstrations instead
of cuRobo-planned ones.

What's left in this directory:

| Path | What it is |
|---|---|
| `task_params.py` | Config surface (`PickPlaceTaskParams`) still used by the Isaac Lab env (`pick_place_env_cfg.py`) and the Mimic env (`pick_place_bimanual_mimic_env.py`). |
| `task_geometry.py` | Shared, Isaac-free task constants (workspace box, table + tray geometry, gripper approach frame). |
| `run_isaaclab_mimic_script.py` | Wrapper that runs IsaacLab's stock `annotate_demos.py` / `generate_dataset.py` against the custom `pick_place_bimanual` task — still valid, but now needs a non-cuRobo source of seed demos to annotate. |
| `params/*.yaml` | Task configs (tray, stack, table modes) consumed by `task_params.py`. |

See `MIMIC_DATASET.md` for the Isaac Lab Mimic scaling pipeline (which never
depended on cuRobo directly — it bridges actions via a `DifferentialIKController`,
not the motion planner). That pipeline is intact; only its cuRobo-scripted
seed-demo source is gone.
