"""Vision domain randomization for push-block distillation (sim2real template).

Code-only DR plus a tiny local texture set (3 PNGs). Scale ranges / add more
textures later without changing the event wiring.
"""

from __future__ import annotations

import math
import random
from pathlib import Path
from typing import TYPE_CHECKING, Sequence

import numpy as np
import torch

import isaaclab.utils.math as math_utils
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

__all__ = [
    "TEXTURE_DIR",
    "list_push_textures",
    "randomize_tiled_camera_pose",
    "randomize_dome_light",
    "randomize_visual_materials",
]

TEXTURE_DIR = Path(__file__).resolve().parents[1] / "assets" / "textures"

# Nominal external camera (must match distill_env_cfg).
_CAM_EYE = np.array([0.55, -0.50, 0.42], dtype=np.float64)
_CAM_TARGET = np.array([0.30, 0.0, 0.05], dtype=np.float64)


def list_push_textures() -> list[str]:
    """Absolute paths to the small template texture pack."""
    if not TEXTURE_DIR.is_dir():
        return []
    return sorted(str(p) for p in TEXTURE_DIR.glob("*.png"))


def _look_at_quat_ros(eye: np.ndarray, target: np.ndarray, up=(0.0, 0.0, 1.0)) -> torch.Tensor:
    """Return wxyz quaternion (ROS optical) as a length-4 tensor."""
    z = target - eye
    z = z / (np.linalg.norm(z) + 1e-12)
    up_a = np.asarray(up, dtype=np.float64)
    x = np.cross(z, up_a)
    x = x / (np.linalg.norm(x) + 1e-12)
    y = np.cross(z, x)
    m = np.stack([x, y, z], axis=1)
    w = math.sqrt(max(1.0 + m[0, 0] + m[1, 1] + m[2, 2], 1e-12)) / 2.0
    quat = np.array(
        [
            w,
            (m[2, 1] - m[1, 2]) / (4 * w),
            (m[0, 2] - m[2, 0]) / (4 * w),
            (m[1, 0] - m[0, 1]) / (4 * w),
        ],
        dtype=np.float32,
    )
    return torch.from_numpy(quat)


def randomize_tiled_camera_pose(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    camera_cfg: SceneEntityCfg = SceneEntityCfg("tiled_camera"),
    pos_range: tuple[float, float] = (-0.03, 0.03),
    rot_deg_range: tuple[float, float] = (-3.0, 3.0),
):
    """Jitter the external tiled camera pose per reset env (sim2real template)."""
    if camera_cfg.name not in env.scene.sensors:
        return
    camera = env.scene.sensors[camera_cfg.name]
    n = len(env_ids)
    device = env.device

    pos_noise = math_utils.sample_uniform(pos_range[0], pos_range[1], (n, 3), device=device)
    # small euler jitter in degrees around the look-at orientation
    rot_noise_deg = math_utils.sample_uniform(rot_deg_range[0], rot_deg_range[1], (n, 3), device=device)

    base_eye = torch.tensor(_CAM_EYE, device=device, dtype=torch.float32)
    base_target = torch.tensor(_CAM_TARGET, device=device, dtype=torch.float32)
    origins = env.scene.env_origins[env_ids]

    eyes = base_eye.unsqueeze(0) + pos_noise
    # keep looking near the workspace; nudge target slightly too
    targets = base_target.unsqueeze(0) + 0.25 * pos_noise

    quats = []
    for i in range(n):
        eye_i = eyes[i].detach().cpu().numpy()
        tgt_i = targets[i].detach().cpu().numpy()
        q = _look_at_quat_ros(eye_i, tgt_i)
        # apply extra euler jitter in camera frame via quat multiply
        droll, dpitch, dyaw = torch.deg2rad(rot_noise_deg[i]).tolist()
        dq = math_utils.quat_from_euler_xyz(
            torch.tensor([droll]), torch.tensor([dpitch]), torch.tensor([dyaw])
        )[0]
        q = math_utils.quat_mul(q.unsqueeze(0), dq.unsqueeze(0))[0]
        quats.append(q)
    orientations = torch.stack(quats, dim=0).to(device=device)

    camera.set_world_poses(
        positions=eyes + origins,
        orientations=orientations,
        env_ids=env_ids,
        convention="ros",
    )


def randomize_dome_light(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    intensity_range: tuple[float, float] = (1500.0, 4500.0),
    color_scale_range: tuple[float, float] = (0.85, 1.15),
    light_prim_path: str = "/World/light",
):
    """Randomize global dome light intensity / tint (shared across envs)."""
    # env_ids unused: dome light is stage-global. Still gated on reset for ADR-style use.
    del env_ids
    try:
        import omni.usd
        from pxr import Gf
    except ImportError:
        return

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(light_prim_path)
    if not prim.IsValid():
        return

    intensity = float(np.random.uniform(*intensity_range))
    tint = [float(np.random.uniform(*color_scale_range)) for _ in range(3)]
    # keep roughly gray-ish but allow mild warm/cool
    color = Gf.Vec3f(min(tint[0], 1.25), min(tint[1], 1.25), min(tint[2], 1.25))

    inten_attr = prim.GetAttribute("inputs:intensity")
    if inten_attr.IsValid():
        inten_attr.Set(intensity)
    color_attr = prim.GetAttribute("inputs:color")
    if color_attr.IsValid():
        color_attr.Set(color)


def _iter_shader_prims(stage, root_path: str):
    """Yield UsdShade-like shader prims under root_path (best-effort)."""
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return
    prefix = root_path.rstrip("/")
    for prim in root.GetAllChildren():
        stack = [prim]
        while stack:
            p = stack.pop()
            stack.extend(list(p.GetAllChildren()))
            path = str(p.GetPath())
            if not path.startswith(prefix):
                continue
            name = p.GetName().lower()
            type_name = p.GetTypeName()
            if "Shader" in type_name or name == "shader" or "omnipbr" in path.lower():
                yield p

def randomize_visual_materials(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    texture_paths: Sequence[str] | None = None,
    table_prim_suffix: str = "/Table",
    object_prim_suffix: str = "/Object",
    box_prim_suffix: str = "/Box",
    apply_textures: bool = True,
):
    """Randomize tint/roughness and optionally swap a small texture pack.

    Best-effort: skips prims/attributes that do not exist so Nucleus table /
    local block-box USDs with different material graphs still run.
    """
    try:
        import omni.usd
        from pxr import Gf, Sdf
    except ImportError:
        return

    if texture_paths is None:
        texture_paths = list_push_textures()
    stage = omni.usd.get_context().get_stage()

    for env_id in env_ids.tolist():
        env_root = f"/World/envs/env_{int(env_id)}"
        for suffix in (table_prim_suffix, object_prim_suffix, box_prim_suffix):
            root = env_root + suffix
            tint = (
                float(np.random.uniform(0.55, 1.0)),
                float(np.random.uniform(0.50, 0.95)),
                float(np.random.uniform(0.45, 0.90)),
            )
            roughness = float(np.random.uniform(0.25, 0.95))
            metallic = float(np.random.uniform(0.0, 0.35))
            tex = random.choice(texture_paths) if (apply_textures and texture_paths) else None

            for shader in _iter_shader_prims(stage, root):
                # MDL / OmniPBR style
                for attr_name, value in (
                    ("inputs:diffuse_tint", Gf.Vec3f(*tint)),
                    ("inputs:reflection_roughness_constant", roughness),
                    ("inputs:metallic_constant", metallic),
                    ("inputs:diffuse_color_constant", Gf.Vec3f(*tint)),
                    ("inputs:diffuseColor", Gf.Vec3f(*tint)),
                ):
                    attr = shader.GetAttribute(attr_name)
                    if attr.IsValid():
                        try:
                            attr.Set(value)
                        except Exception:
                            pass
                if tex is not None:
                    for attr_name in ("inputs:diffuse_texture", "inputs:diffuse_color_texture"):
                        attr = shader.GetAttribute(attr_name)
                        if attr.IsValid():
                            try:
                                attr.Set(Sdf.AssetPath(tex))
                            except Exception:
                                pass
                        else:
                            # create input if this looks like an OmniPBR shader
                            try:
                                from pxr import UsdShade

                                usd_shader = UsdShade.Shader(shader)
                                if usd_shader:
                                    usd_shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
                                        Sdf.AssetPath(tex)
                                    )
                            except Exception:
                                pass
