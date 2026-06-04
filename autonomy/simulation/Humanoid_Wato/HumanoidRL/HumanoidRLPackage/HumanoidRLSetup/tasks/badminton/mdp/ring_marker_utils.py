"""Flat annulus ring meshes for the badminton intercept target visualizer."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import MISSING

import numpy as np
import trimesh
from pxr import Usd

import isaacsim.core.utils.prims as prim_utils
import isaaclab.sim as sim_utils
from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.sim.spawners.meshes.meshes import _spawn_mesh_geom_from_mesh
from isaaclab.sim.spawners.meshes.meshes_cfg import MeshCfg
from isaaclab.utils import configclass

# Ring bands from center outward, matching the reference photo palette.
INTERCEPT_RING_SPECS: tuple[tuple[str, float, float, tuple[float, float, float]], ...] = (
    ("ring_red", 0.0, 0.035, (1.0, 0.0, 0.0)),
    ("ring_yellow", 0.035, 0.070, (1.0, 1.0, 0.0)),
    ("ring_green", 0.070, 0.110, (0.0, 1.0, 0.0)),
    ("ring_blue", 0.110, 0.160, (0.0, 0.0, 1.0)),
)


def make_annulus_trimesh(inner_radius: float, outer_radius: float, segments: int = 64) -> trimesh.Trimesh:
    """Flat annulus in the XY plane (normal +Z), suitable for a tilted intercept disk."""
    angles = np.linspace(0.0, 2.0 * np.pi, segments, endpoint=False)
    cos_a = np.cos(angles)
    sin_a = np.sin(angles)

    outer = np.stack([outer_radius * cos_a, outer_radius * sin_a, np.zeros(segments)], axis=1)
    if inner_radius <= 1.0e-6:
        center = np.array([[0.0, 0.0, 0.0]])
        vertices = np.vstack([center, outer])
        faces = []
        for i in range(segments):
            j = (i + 1) % segments
            faces.append([0, i + 1, j + 1])
        return trimesh.Trimesh(vertices=vertices, faces=np.asarray(faces))

    inner = np.stack([inner_radius * cos_a, inner_radius * sin_a, np.zeros(segments)], axis=1)
    vertices = np.vstack([outer, inner])
    faces = []
    for i in range(segments):
        j = (i + 1) % segments
        o0, o1 = i, j
        i0, i1 = segments + i, segments + j
        faces.append([o0, o1, i1])
        faces.append([o0, i1, i0])
    return trimesh.Trimesh(vertices=vertices, faces=np.asarray(faces))


@configclass
class AnnulusMeshCfg(MeshCfg):
    """Visual-only flat annulus mesh."""

    func: Callable[..., Usd.Prim] = MISSING
    inner_radius: float = 0.0
    outer_radius: float = 0.1
    segments: int = 64
    visual_material: sim_utils.VisualMaterialCfg | None = None


def spawn_annulus(
    prim_path: str,
    cfg: AnnulusMeshCfg,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
) -> Usd.Prim:
    """Spawn a flat annulus mesh prim for debug visualization."""
    mesh = make_annulus_trimesh(cfg.inner_radius, cfg.outer_radius, cfg.segments)
    _spawn_mesh_geom_from_mesh(prim_path, cfg, mesh, translation, orientation)
    return prim_utils.get_prim_at_path(prim_path)


def _ring_material(color: tuple[float, float, float], emissive_scale: float = 0.35) -> sim_utils.PreviewSurfaceCfg:
    emissive = tuple(min(1.0, c * emissive_scale + 0.15) for c in color)
    return sim_utils.PreviewSurfaceCfg(
        diffuse_color=color,
        emissive_color=emissive,
        opacity=0.92,
    )


def build_intercept_target_visualizer_cfg() -> VisualizationMarkersCfg:
    """Concentric flat rings + white center dot, like the badminton intercept reference."""
    markers: dict[str, AnnulusMeshCfg | sim_utils.SphereCfg] = {}
    for name, inner_r, outer_r, color in INTERCEPT_RING_SPECS:
        markers[name] = AnnulusMeshCfg(
            func=spawn_annulus,
            inner_radius=inner_r,
            outer_radius=outer_r,
            visual_material=_ring_material(color),
        )
    markers["center_dot"] = sim_utils.SphereCfg(
        radius=0.015,
        visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=(1.0, 1.0, 1.0),
            emissive_color=(0.85, 0.85, 0.85),
            opacity=1.0,
        ),
    )
    return VisualizationMarkersCfg(
        prim_path="/Visuals/Command/intercept_target",
        markers=markers,
    )


# Marker prototype order in VisualizationMarkers (must match dict insertion order above).
INTERCEPT_MARKER_NAMES: tuple[str, ...] = tuple(name for name, _, _, _ in INTERCEPT_RING_SPECS) + ("center_dot",)
NUM_INTERCEPT_MARKERS = len(INTERCEPT_MARKER_NAMES)
