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


def print_joint_limits_from_usd(robot):
    """Print joint position limits from the USD-loaded articulation (Isaac Sim).
    Use this to check if joint 0 (shoulder_flexion_extension) has limits that could block motion.
    """
    names = getattr(robot.data, "joint_names", None)
    limits = getattr(robot.data, "default_joint_pos_limits", None) or getattr(robot.data, "joint_pos_limits", None)
    if names is None or limits is None:
        print("[joint limits] joint_names or (default_)joint_pos_limits not found on robot.data")
        return
    # limits shape: (num_instances, num_joints, 2) or (num_joints, 2) -> [lower, upper]
    lim = limits.cpu().numpy()
    if lim.ndim == 3:
        lim = lim[0]
    low, high = lim[:, 0], lim[:, 1]
    print("[joint limits] From USD (robot.data.default_joint_pos_limits or joint_pos_limits):")
    for i, name in enumerate(names):
        marker = "  <-- joint 0 (shoulder_flexion)" if i == 0 else ""
        print(f"  [{i}] {name}: lower={low[i]:.4f}, upper={high[i]:.4f}{marker}")
    if hasattr(robot.data, "soft_joint_pos_limits") and robot.data.soft_joint_pos_limits is not None:
        soft = robot.data.soft_joint_pos_limits[0].cpu().numpy()
        if soft.ndim == 2:
            print("[joint limits] soft_joint_pos_limits (first 6):", soft[:6])


def run_simulator(
    sim: sim_utils.SimulationContext,
    scene: InteractiveScene,
    pose_sequence: list[tuple[np.ndarray, np.ndarray, float]],
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

    # Map fingertip body names (from URDF) to Isaac body indices once.
    fingertip_body_ids: list[int | None] = []
    for name in FINGERTIP_BODIES:
        ids, _ = robot.find_bodies(name, preserve_order=True)
        fingertip_body_ids.append(ids[0] if ids else None)

    pose_idx = 0
    step_count = 0
    n_poses = len(pose_sequence)
    print(f"pose 1/{n_poses}: IK err {pose_sequence[0][2]:.4f} m")

    while simulation_app.is_running():
        # Advance to next pose when we've held long enough
        if step_count >= steps_per_pose:
            step_count = 0
            pose_idx = (pose_idx + 1) % n_poses
            ik_err = pose_sequence[pose_idx][2]
            print(f"pose {pose_idx + 1}/{n_poses}: IK err {ik_err:.4f} m")

        # Interpolate between current and next pose so motion is smooth (no snap/reset)
        next_idx = (pose_idx + 1) % n_poses
        alpha = step_count / steps_per_pose if steps_per_pose > 0 else 1.0
        alpha = min(alpha, 1.0)

        qpos_cur, targets_cur, _ = pose_sequence[pose_idx]
        qpos_next, targets_next, _ = pose_sequence[next_idx]
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

        # After the first sim step of each pose, print per-finger errors in Isaac world frame.
        if step_count == 1:
            targets_np = targets_world.cpu().numpy()
            max_finger_err = 0.0
            print("[Isaac] Per-finger position error at current pose (m):")
            for j, (name, body_id) in enumerate(zip(FINGERTIP_BODIES, fingertip_body_ids)):
                if body_id is None:
                    print(f"  {name}: (body not found)")
                    continue
                tip_pos = robot.data.body_pos_w[0, body_id, :].cpu().numpy()
                target_pos = targets_np[j]
                err_vec = tip_pos - target_pos
                err_norm = float(np.linalg.norm(err_vec))
                max_finger_err = max(max_finger_err, err_norm)
                print(f"  {name}: err={err_norm:.4f}, tip={tip_pos}, target={target_pos}")
            print(f"[Isaac] Max fingertip error this pose: {max_finger_err:.4f} m")


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
    range_m = 0.14  # motion amplitude

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
    n_poses = len(all_offsets)
    for i, offset in enumerate(all_offsets):
        targets_mjc = {name: pos + offset for name, pos in default_positions.items()}
        data.qpos[:] = 0.0
        qpos, converged, max_err = solve_fingertip_ik(
            model, data, targets_mjc,
            damping=5e-4, step=0.25, max_iter=3000, tol=5e-3,
        )
        target_positions_isaac = np.stack([targets_mjc[name] for name in FINGERTIP_BODIES], axis=0).astype(np.float32)
        pose_sequence.append((qpos.copy(), target_positions_isaac, max_err))
        print(f"pose {i + 1}/{n_poses}: IK err {max_err:.4f} m, qpos={qpos}")

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