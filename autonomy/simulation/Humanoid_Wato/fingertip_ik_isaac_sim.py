import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Fingertip IK in Isaac Sim: 5 EE targets, 21-DOF arm+hand")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import numpy as np
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

from HumanoidRL.HumanoidRLPackage.HumanoidRLSetup.modelCfg.humanoid import ARM_CFG

from arm_assembly.fingertip_ik import (
    load_model,
    solve_fingertip_ik,
    get_fingertip_positions,
    FINGERTIP_BODIES,
)

FINGERTIP_BODY_PATTERNS = [
    ("IP_THUMB_v1_.*", "thumb"),
    ("DIP_INDEX_v1_.*", "index"),
    ("DIP_MIDDLE_v1_.*", "middle"),
    ("DIP_RING_v1_.*", "ring"),
    ("DIP_PINKY_v1_.*", "pinky"),
]


@configclass
class FingertipIKSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )
    robot = ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


def run_simulator(
    sim: sim_utils.SimulationContext,
    scene: InteractiveScene,
    pose_sequence: list[tuple[np.ndarray, np.ndarray]],
    hold_seconds: float = 1.5,
):
    robot = scene["robot"]
    robot_entity_cfg = SceneEntityCfg("robot", joint_names=[".*"], body_names=[".*"])
    robot_entity_cfg.resolve(scene)

    sim_dt = sim.get_physics_dt()
    steps_per_pose = max(1, int(hold_seconds / sim_dt))
    joint_vel = robot.data.default_joint_vel.clone()

    # Markers for the 5 target EE positions (spheres in Isaac)
    target_marker_cfg = VisualizationMarkersCfg(
        prim_path="/Visuals/fingertip_targets",
        markers={
            "target": sim_utils.SphereCfg(
                radius=0.015,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.2, 0.2)),
            ),
        },
    )
    target_markers = VisualizationMarkers(target_marker_cfg)

    # Resolve Isaac fingertip body IDs for printing (robot body_pos_w)
    fingertip_body_ids = []
    for pattern, label in FINGERTIP_BODY_PATTERNS:
        ids, names = robot.find_bodies(pattern, preserve_order=True)
        if ids:
            fingertip_body_ids.append((ids[0], label, names[0] if names else pattern))
        else:
            fingertip_body_ids.append((None, label, pattern))
    if not any(ids is not None for ids, _, _ in fingertip_body_ids):
        print("[WARN] No fingertip bodies found on robot; cannot print Isaac fingertip positions.")

    pose_idx = 0
    step_count = 0
    print_finger_debug = True  # print once after first pose so we can compare scale

    while simulation_app.is_running():
        # Advance to next pose when we've held long enough
        if step_count >= steps_per_pose:
            step_count = 0
            pose_idx = (pose_idx + 1) % len(pose_sequence)
            if (pose_idx + 1) % 20 == 0 or pose_idx == 0:
                print(f"[Cycle] Step {pose_idx + 1}/{len(pose_sequence)}")
            print_finger_debug = True

        # Interpolate between current and next pose so motion is smooth (no snap/reset)
        next_idx = (pose_idx + 1) % len(pose_sequence)
        alpha = step_count / steps_per_pose if steps_per_pose > 0 else 1.0
        alpha = min(alpha, 1.0)

        qpos_cur, targets_cur = pose_sequence[pose_idx]
        qpos_next, targets_next = pose_sequence[next_idx]
        qpos_interp = (1.0 - alpha) * qpos_cur + alpha * qpos_next
        targets_interp = (1.0 - alpha) * targets_cur + alpha * targets_next

        # Write interpolated state every frame so robot and markers stay in sync (no PD lag)
        joint_pos_tensor = torch.tensor(qpos_interp, dtype=torch.float32, device=sim.device)
        robot.write_joint_state_to_sim(joint_pos_tensor, joint_vel)
        targets_world = torch.tensor(targets_interp, dtype=torch.float32, device=sim.device) + scene.env_origins[0]
        target_markers.visualize(translations=targets_world)
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)
        step_count += 1

        # Print Isaac fingertip positions vs marker positions once per pose to verify scale
        if print_finger_debug and step_count == 1:
            print_finger_debug = False
            print("[Isaac] Robot fingertip positions (world, same units as scene):")
            for body_id, label, name in fingertip_body_ids:
                if body_id is not None:
                    pos = robot.data.body_pos_w[0, body_id, :].cpu().numpy()
                    print(f"  {label} ({name}): {pos}")
                else:
                    print(f"  {label}: (body not found)")
            marker_pos = targets_world.cpu().numpy()  # (5, 3)
            print("[Isaac] Marker positions we're drawing (targets_world):")
            for j, (_, label) in enumerate(FINGERTIP_BODY_PATTERNS):
                p = marker_pos[j] if marker_pos.ndim > 1 else marker_pos
                print(f"  {label}: {p}")


def main():
    import mujoco

    # ---- Run IK for several 5-point target sets ----
    model = load_model()
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    default_positions = get_fingertip_positions(model, data)
    if len(default_positions) != 5:
        raise RuntimeError("Could not get all 5 fingertip positions from model.")

    # Build a list of (dx, dy, dz) offsets in meters; each motion goes there and back.
    n = 10  # steps per half (out or back)
    range_m = 0.04  # motion amplitude

    def lin_there_back(start, end):
        out = np.linspace(start, end, n, dtype=np.float64)
        back = np.linspace(end, start, n, dtype=np.float64)
        return np.concatenate([out, back[1:]])

    # 1. Vertical: Z down then up
    z_off = lin_there_back(0.0, -range_m)
    offsets_vertical = np.column_stack([np.zeros_like(z_off), np.zeros_like(z_off), z_off])

    # 2. Horizontal X: move in X then back
    x_off = lin_there_back(0.0, -range_m)
    offsets_x = np.column_stack([x_off, np.zeros_like(x_off), np.zeros_like(x_off)])

    # 3. Horizontal Y: move in Y then back
    y_off = lin_there_back(0.0, range_m)
    offsets_y = np.column_stack([np.zeros_like(y_off), y_off, np.zeros_like(y_off)])

    # 4. Diagonal: X and Z together
    d_off = lin_there_back(0.0, -range_m * 0.8)
    offsets_diag = np.column_stack([d_off, np.zeros_like(d_off), d_off])

    all_offsets = np.vstack([offsets_vertical, offsets_x, offsets_y, offsets_diag])

    pose_sequence = []
    for i, offset in enumerate(all_offsets):
        targets_mjc = {name: pos + offset for name, pos in default_positions.items()}
        data.qpos[:] = 0.0
        qpos, converged, max_err = solve_fingertip_ik(
            model, data, targets_mjc,
            damping=1e-3, step=0.35, max_iter=1000, tol=1.2e-2,
        )
        target_positions_isaac = np.stack([targets_mjc[name] for name in FINGERTIP_BODIES], axis=0).astype(np.float32)
        pose_sequence.append((qpos.copy(), target_positions_isaac))
        if (i + 1) % 15 == 0 or i == 0:
            print(f"[IK] Motion step {i + 1}/{len(all_offsets)} offset={offset}: converged={converged}, max_err={max_err:.4f} m")

    # ---- Isaac Sim ----
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])

    scene_cfg = FingertipIKSceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    sim.reset()
    print("[INFO] Isaac Sim ready. Press Play: targets move vertical, then X, then Y, then diagonal; fingers should follow.")
    print(f"[INFO] env_origins[0] (marker offset) = {scene.env_origins[0].cpu().numpy()}")

    run_simulator(sim, scene, pose_sequence, hold_seconds=0.08)


if __name__ == "__main__":
    main()
    simulation_app.close()

# PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p fingertip_ik_isaac_sim.py