"""Domain randomization for SO101 vial-task teleop (NVIDIA workshop parity).

Works with Isaac Lab ``InteractiveScene`` (no Gym env required). Call
``apply_vial_task_domain_rand(scene)`` at episode start / world reset.
"""

from __future__ import annotations

import glob
import math
import os
import random
from typing import Any

import torch

import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
from isaaclab.sim import get_current_stage
from isaacsim.core.prims import XFormPrim
from pxr import Gf, Sdf

ROBOT_COLORS: dict[str, tuple[float, float, float]] = {
    "orange": (0.876, 0.317, 0.132),
    "teal": (0.0, 0.8, 0.502),
    "white": (0.95, 0.95, 0.95),
    "black": (0.08, 0.08, 0.08),
}

_DEFAULT_POSES_CACHE: dict[str, dict[str, Any]] = {}


def _env_ids(scene) -> torch.Tensor:
    return torch.tensor([0], device=scene.device)


def _resolve_prim_path(pattern: str, env_index: int = 0) -> str:
    return pattern.replace("{ENV_REGEX_NS}", f"/World/envs/env_{env_index}")


def randomize_robot_color(scene, color_names: list[str] | None = None) -> None:
    """Random robot shell color (workshop ``randomize_robot_color``)."""
    if color_names is None:
        color_names = list(ROBOT_COLORS.keys())
    robot = scene["robot"]
    color = ROBOT_COLORS[color_names[random.randint(0, len(color_names) - 1)]]
    material_path = _resolve_prim_path(robot.cfg.prim_path) + "/Looks/material_a_3d_printed/Shader"
    prims = sim_utils.find_matching_prims(material_path)
    if not prims:
        return
    with Sdf.ChangeBlock():
        prims[0].GetAttribute("inputs:diffuse_color_constant").Set(color)


def randomize_mat_rotation(
    scene,
    mat_name: str = "mat",
    yaw_range: tuple[float, float] = (-0.3, 0.3),
) -> None:
    asset = scene[mat_name]
    prim_path = asset.prim_paths[0]
    yaw = math_utils.sample_uniform(yaw_range[0], yaw_range[1], (1,), device=scene.device)
    roll = torch.zeros_like(yaw)
    pitch = torch.zeros_like(yaw)
    base_yaw = torch.full_like(yaw, math.pi / 2)
    orientations = math_utils.quat_from_euler_xyz(roll, pitch, base_yaw + yaw)
    xform = XFormPrim(prim_paths_expr=prim_path)
    with Sdf.ChangeBlock():
        xform.set_local_poses(orientations=orientations)


def randomize_light_exposure(scene, light_name: str = "lightbox_light", exposure_range: tuple[float, float] = (-3.0, 1.0)) -> None:
    if light_name not in scene:
        return
    asset = scene[light_name]
    exposure = math_utils.sample_uniform(exposure_range[0], exposure_range[1], (1,), device="cpu").item()
    stage = get_current_stage()
    with Sdf.ChangeBlock():
        prim = stage.GetPrimAtPath(asset.prim_paths[0])
        if prim.IsValid():
            attr = prim.GetAttribute("inputs:exposure")
            if attr.IsValid():
                attr.Set(exposure)


def randomize_camera_focal_length(
    scene,
    camera_name: str = "camera_ego",
    focal_length_range: tuple[float, float] = (12.0, 15.0),
) -> None:
    if camera_name not in scene:
        return
    camera = scene[camera_name]
    camera_prim_path = camera.cfg.prim_path.replace("{ENV_REGEX_NS}", "/World/envs/env_.*")
    focal_length = math_utils.sample_uniform(
        focal_length_range[0], focal_length_range[1], (1,), device="cpu"
    ).item()
    with Sdf.ChangeBlock():
        for prim in sim_utils.find_matching_prims(camera_prim_path):
            if prim.IsValid():
                attr = prim.GetAttribute("focalLength")
                if attr.IsValid():
                    attr.Set(focal_length)


def randomize_camera_pose(
    scene,
    mount_pattern: str = "{ENV_REGEX_NS}/LightStudio/LightBox/camera_mount",
    pos_range: dict[str, tuple[float, float]] | None = None,
    rot_range: dict[str, tuple[float, float]] | None = None,
) -> None:
    pos_range = pos_range or {"x": (-0.02, 0.02), "y": (-0.02, 0.02), "z": (-0.01, 0.01)}
    rot_range = rot_range or {"roll": (-0.05, 0.05), "pitch": (-0.05, 0.05), "yaw": (-0.05, 0.05)}
    prim_path = mount_pattern.replace("{ENV_REGEX_NS}", "/World/envs/env_.*")
    prims = sim_utils.find_matching_prims(prim_path)
    if not prims:
        return

    if mount_pattern not in _DEFAULT_POSES_CACHE:
        prim = prims[0]
        default_pos = (0.0, 0.0, 0.0)
        default_quat = (1.0, 0.0, 0.0, 0.0)
        translate_attr = prim.GetAttribute("xformOp:translate")
        orient_attr = prim.GetAttribute("xformOp:orient")
        if translate_attr.IsValid():
            val = translate_attr.Get()
            if val is not None:
                default_pos = (val[0], val[1], val[2])
        if orient_attr.IsValid():
            val = orient_attr.Get()
            if val is not None:
                default_quat = (
                    val.GetReal(),
                    val.GetImaginary()[0],
                    val.GetImaginary()[1],
                    val.GetImaginary()[2],
                )
        _DEFAULT_POSES_CACHE[mount_pattern] = {"pos": default_pos, "quat": default_quat}

    base_pos = _DEFAULT_POSES_CACHE[mount_pattern]["pos"]
    base_quat = _DEFAULT_POSES_CACHE[mount_pattern]["quat"]

    x = base_pos[0] + math_utils.sample_uniform(*pos_range.get("x", (0, 0)), (1,), device="cpu").item()
    y = base_pos[1] + math_utils.sample_uniform(*pos_range.get("y", (0, 0)), (1,), device="cpu").item()
    z = base_pos[2] + math_utils.sample_uniform(*pos_range.get("z", (0, 0)), (1,), device="cpu").item()

    roll = math_utils.sample_uniform(*rot_range.get("roll", (0, 0)), (1,), device="cpu").item()
    pitch = math_utils.sample_uniform(*rot_range.get("pitch", (0, 0)), (1,), device="cpu").item()
    yaw = math_utils.sample_uniform(*rot_range.get("yaw", (0, 0)), (1,), device="cpu").item()
    delta_quat = math_utils.quat_from_euler_xyz(
        torch.tensor([roll]), torch.tensor([pitch]), torch.tensor([yaw])
    )[0]
    base_quat_tensor = torch.tensor([base_quat])
    final_quat = math_utils.quat_mul(base_quat_tensor, delta_quat.unsqueeze(0))[0]

    with Sdf.ChangeBlock():
        for prim in prims:
            if not prim.IsValid():
                continue
            translate_attr = prim.GetAttribute("xformOp:translate")
            if translate_attr.IsValid():
                translate_attr.Set(Gf.Vec3d(x, y, z))
            orient_attr = prim.GetAttribute("xformOp:orient")
            if orient_attr.IsValid():
                orient_attr.Set(
                    Gf.Quatd(
                        final_quat[0].item(),
                        final_quat[1].item(),
                        final_quat[2].item(),
                        final_quat[3].item(),
                    )
                )


def _random_asset_pose(scene, asset, pose_range: dict, pos_offset: dict | None = None) -> tuple[torch.Tensor, torch.Tensor]:
    env_ids = _env_ids(scene)
    pos_offset = pos_offset or {}
    root_states = asset.data.default_root_state[env_ids].clone()
    pos_offset_vec = torch.tensor(
        [pos_offset.get(k, 0.0) for k in ("x", "y", "z")],
        device=asset.device,
    )
    range_list = [pose_range.get(k, (0.0, 0.0)) for k in ("x", "y", "z", "roll", "pitch", "yaw")]
    ranges = torch.tensor(range_list, device=asset.device)
    rand_samples = math_utils.sample_uniform(
        ranges[:, 0], ranges[:, 1], (len(env_ids), 6), device=asset.device
    )
    positions = root_states[:, 0:3] + scene.env_origins[env_ids] + rand_samples[:, 0:3] + pos_offset_vec
    orientations_delta = math_utils.quat_from_euler_xyz(
        rand_samples[:, 3], rand_samples[:, 4], rand_samples[:, 5]
    )
    orientations = math_utils.quat_mul(root_states[:, 3:7], orientations_delta)
    asset.write_root_pose_to_sim(torch.cat([positions, orientations], dim=-1), env_ids=env_ids)
    return positions, orientations


def reset_vials_rack(
    scene,
    *,
    vial_names: tuple[str, ...] = ("vial_1", "vial_2", "vial_3"),
    rack_name: str = "rack_left",
    rack_pose_range: dict | None = None,
    vial_pose_range: dict | None = None,
    fixed_vial_z: float = 0.05,
    rack_placement_prob: float = 0.33,
) -> None:
    """Randomize vial and rack poses (workshop ``reset_vials_rack``)."""
    rack_pose_range = rack_pose_range or {"x": (-0.04, 0.04), "y": (-0.01, 0.01), "yaw": (-0.5, 0.5)}
    vial_pose_range = vial_pose_range or {
        "x": (-0.04, 0.04),
        "y": (-0.01, 0.01),
        "roll": (-0.3, 0.3),
        "yaw": (0.0, 0.0),
    }
    env_ids = _env_ids(scene)
    vial_objects = [scene[name] for name in vial_names]
    rack = scene[rack_name]

    new_rack_positions, new_rack_orientations = _random_asset_pose(scene, rack, rack_pose_range)
    zero_velocity = torch.zeros((len(env_ids), 6), device=rack.device)
    rack.write_root_velocity_to_sim(zero_velocity, env_ids=env_ids)

    placed_indices: list[int] = []
    if random.random() < rack_placement_prob:
        placed_indices.append(random.randint(0, len(vial_objects) - 1))

    slots_xform = XFormPrim(prim_paths_expr=f"{rack.cfg.prim_path}/Body1/Mesh/top_*")
    total_slots = len(slots_xform.prims)

    if placed_indices and total_slots > 0:
        slot_positions_local, slot_orientations_local = slots_xform.get_local_poses()
        for vial_idx in placed_indices:
            vial = vial_objects[vial_idx]
            slot_idx = random.randint(0, total_slots - 1)
            slot_position_local = slot_positions_local[slot_idx].unsqueeze(0).repeat(len(env_ids), 1)
            slot_orientation_local = slot_orientations_local[slot_idx].unsqueeze(0).repeat(len(env_ids), 1)
            slot_position, slot_orientation = math_utils.combine_frame_transforms(
                new_rack_positions, new_rack_orientations, slot_position_local, slot_orientation_local
            )
            vial.write_root_pose_to_sim(torch.cat([slot_position, slot_orientation], dim=-1), env_ids=env_ids)
            vial.write_root_velocity_to_sim(zero_velocity, env_ids=env_ids)

    pose_range_z_fixed = {**vial_pose_range, "z": (0.0, 0.0)}
    for i, vial in enumerate(vial_objects):
        if i in placed_indices:
            continue
        default_z = vial.data.default_root_state[env_ids[0], 2].item()
        pos_offset = {"z": fixed_vial_z - default_z}
        _random_asset_pose(scene, vial, pose_range_z_fixed, pos_offset)
        vial.write_root_velocity_to_sim(zero_velocity, env_ids=env_ids)


def randomize_sky_light(
    scene,
    hdri_dir: str,
    sky_name: str = "sky_light",
    exposure_range: tuple[float, float] = (-4.0, 3.0),
    temperature_range: tuple[float, float] = (2500.0, 9500.0),
) -> None:
    """Random HDRI dome light (workshop ``VialsToRackEventDRCfg.reset_sky_light``)."""
    if sky_name not in scene or not os.path.isdir(hdri_dir):
        return
    textures = glob.glob(os.path.join(hdri_dir, "*.exr"))
    if not textures:
        return

    asset = scene[sky_name]
    stage = get_current_stage()
    exposure = math_utils.sample_uniform(exposure_range[0], exposure_range[1], (1,), device="cpu").item()
    temperature = math_utils.sample_uniform(temperature_range[0], temperature_range[1], (1,), device="cpu").item()
    texture_path = textures[random.randint(0, len(textures) - 1)]

    yaw_mapping_path = os.path.join(hdri_dir, "yaw_mapping.yaml")
    yaw_range = (0.0, 0.0)
    if os.path.isfile(yaw_mapping_path):
        import yaml

        with open(yaw_mapping_path, encoding="utf-8") as f:
            mapping = yaml.safe_load(f) or {}
        entry = mapping.get(os.path.basename(texture_path))
        if entry:
            yaw_range = (float(entry[0]), float(entry[1]))

    env_ids = _env_ids(scene)
    ranges = torch.tensor([(0.0, 0.0), (0.0, 0.0), yaw_range], device=scene.device)
    rand_samples = math_utils.sample_uniform(ranges[:, 0], ranges[:, 1], (len(env_ids), 3), device=scene.device)
    orientations = math_utils.quat_from_euler_xyz(rand_samples[:, 0], rand_samples[:, 1], rand_samples[:, 2])
    xform = XFormPrim(prim_paths_expr=asset.prim_paths[0])

    with Sdf.ChangeBlock():
        xform.set_local_poses(orientations=orientations)
        prim = stage.GetPrimAtPath(asset.prim_paths[0])
        if prim.IsValid():
            prim.GetAttribute("inputs:exposure").Set(exposure)
            prim.GetAttribute("inputs:colorTemperature").Set(temperature)
            prim.GetAttribute("inputs:texture:file").Set(Sdf.AssetPath(texture_path))


def apply_vial_task_domain_rand(
    scene,
    *,
    hdri_dir: str | None = None,
    full_dr: bool = True,
    randomize_props: bool = True,
    randomize_lighting: bool = True,
    randomize_cameras: bool = True,
) -> None:
    """Apply domain randomization for a new episode.

    ``full_dr=True`` matches workshop ``VialsToRackDREnvCfg``:
    mat yaw ±17°, HDRI sky, robot color, vial/rack layout, camera DR.
    """
    mat_yaw = (-0.3, 0.3) if full_dr else (-0.1, 0.1)

    if randomize_props:
        reset_vials_rack(scene)
        randomize_mat_rotation(scene, yaw_range=mat_yaw)
        if full_dr:
            randomize_robot_color(scene)

    if randomize_lighting:
        randomize_light_exposure(scene)
        if full_dr and hdri_dir and "sky_light" in scene:
            randomize_sky_light(scene, hdri_dir)

    if randomize_cameras:
        randomize_camera_focal_length(scene)
        randomize_camera_pose(scene)
