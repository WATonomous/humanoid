"""Shared setup helpers for SO101 leader/keyboard teleop scripts."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

_IL_PKG = Path(__file__).resolve().parents[3] / "il"


def ensure_il_on_path() -> Path:
    if str(_IL_PKG) not in sys.path:
        sys.path.insert(0, str(_IL_PKG))
    return _IL_PKG


def load_record_schema(schema_arg: str) -> dict[str, Any] | None:
    """Load YAML schema when recording or when an explicit schema path is given."""
    ensure_il_on_path()
    from humanoid_il.record_utils import resolve_config_path
    from humanoid_il.schema import load_yaml

    path = resolve_config_path(schema_arg, anchor=_IL_PKG)
    return load_yaml(path)


def prepare_launcher_args(args_cli) -> None:
    """Set Isaac Lab launcher flags (cameras) from CLI + schema."""
    ensure_il_on_path()
    from humanoid_il.so101_cameras import schema_needs_cameras

    if args_cli.cameras:
        args_cli.enable_cameras = True
        return
    if args_cli.record or args_cli.schema != str(_IL_PKG / "config" / "dataset_schema_so101_sim.yaml"):
        cfg = load_record_schema(args_cli.schema)
        if cfg and schema_needs_cameras(cfg):
            args_cli.enable_cameras = True


def build_sim_context(args_cli):
    from so101_cfg import vial_task_render_cfg

    vial = args_cli.scene == "vial"
    use_dr = vial and args_cli.domain_rand
    render = vial_task_render_cfg(domain_rand=use_dr) if vial else None
    import isaaclab.sim as sim_utils

    sim_cfg = sim_utils.SimulationCfg(
        dt=0.01,
        device=args_cli.device,
        render=render if render is not None else sim_utils.RenderCfg(),
    )
    sim = sim_utils.SimulationContext(sim_cfg)
    if vial:
        sim.set_camera_view([0.8, 0.8, 0.6], [0.22, 0.0, 0.08])
    else:
        sim.set_camera_view([1.2, 1.2, 0.8], [0.0, 0.0, 0.3])
    return sim


def build_scene(args_cli):
    import vial_task_assets as vta
    from isaaclab.scene import InteractiveScene
    from so101_cfg import (
        SO101SceneCfg,
        SO101VialTaskDRSceneCfg,
        SO101VialTaskDRVisionSceneCfg,
        SO101VialTaskSceneCfg,
        SO101VialTaskVisionSceneCfg,
        configure_scene_robot,
        configure_vision_cameras,
    )

    robot_kind = getattr(args_cli, "robot", "follower")

    if args_cli.scene == "vial":
        vta.assert_assets_present()
        if args_cli.domain_rand:
            vta.assert_hdri_for_domain_rand()
        use_cameras = getattr(args_cli, "enable_cameras", False) or args_cli.cameras
        if args_cli.domain_rand and use_cameras:
            scene_cfg = SO101VialTaskDRVisionSceneCfg(num_envs=1, env_spacing=2.0)
        elif args_cli.domain_rand:
            scene_cfg = SO101VialTaskDRSceneCfg(num_envs=1, env_spacing=2.0)
        elif use_cameras:
            scene_cfg = SO101VialTaskVisionSceneCfg(num_envs=1, env_spacing=2.0)
        else:
            scene_cfg = SO101VialTaskSceneCfg(num_envs=1, env_spacing=2.0)
    else:
        scene_cfg = SO101SceneCfg(num_envs=1, env_spacing=2.0)

    scene_cfg = configure_scene_robot(scene_cfg, robot_kind)
    if getattr(args_cli, "enable_cameras", False) or args_cli.cameras:
        scene_cfg = configure_vision_cameras(scene_cfg, robot_kind)
    return InteractiveScene(scene_cfg)


def maybe_apply_domain_rand(scene, args_cli) -> None:
    if not args_cli.domain_rand or args_cli.scene != "vial":
        return
    ensure_il_on_path()
    import vial_task_assets as vta
    from humanoid_il.so101_domain_rand import apply_vial_task_domain_rand

    hdri_dir = vta.HDRI_DIR if Path(vta.HDRI_DIR).is_dir() else None
    apply_vial_task_domain_rand(scene, hdri_dir=hdri_dir, full_dr=True)
    print("[INFO] Applied full vial-task domain randomization (VialsToRackDR parity).")


def capture_record_images(scene, record_session) -> dict | None:
    if record_session is None or not record_session.image_keys:
        return None
    ensure_il_on_path()
    from humanoid_il.so101_cameras import capture_rgb_images, schema_image_keys_to_scene

    key_map = schema_image_keys_to_scene(record_session.cfg)
    return capture_rgb_images(scene, key_map)
