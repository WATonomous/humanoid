"""
wato_hand_isaaclab_teleop.py
============================
Isaac Lab simulation that drives the arm_assembly hand joints in real-time
by reading /tmp/wato_joints.json written by wato_hand_ros2_node.py.

No rclpy required — works with isaaclab.sh's own Python interpreter.

Usage (in the Docker container):
  # Terminal 1: rosbridge already running
  # Terminal 2: wato_hand_ros2_node.py already running (writes /tmp/wato_joints.json)
  # Terminal 3:
  cd /workspace/isaaclab/humanoid/autonomy/simulation/Humanoid_Wato
  PYTHONPATH=/workspace/isaaclab/humanoid/autonomy/simulation/Humanoid_Wato \
    /workspace/isaaclab/isaaclab.sh -p wato_hand_isaaclab_teleop.py
"""

import argparse
import json
import os
import time

from isaaclab.app import AppLauncher

# ── AppLauncher must come before any Isaac/USD imports ───────────────────────
parser = argparse.ArgumentParser(description="Wato Hand Isaac Lab Teleop")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ── Now safe to import Isaac / torch ─────────────────────────────────────────
import torch                                        # noqa: E402

import isaaclab.sim as sim_utils                   # noqa: E402
from isaaclab.assets import AssetBaseCfg           # noqa: E402
from isaaclab.managers import SceneEntityCfg       # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils import configclass             # noqa: E402

from HumanoidRL.HumanoidRLPackage.HumanoidRLSetup.modelCfg.humanoid import ARM_CFG  # noqa: E402

# ── Shared file written by wato_hand_ros2_node.py ────────────────────────────
JOINT_FILE = "/tmp/wato_joints.json"
HAND_TIMEOUT = 2.0  # seconds before hand is considered lost


def read_joint_file() -> dict[str, float]:
    """Read the latest joint angles. Returns {} if file missing/corrupt."""
    try:
        with open(JOINT_FILE, "r") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def is_hand_visible(hand_dict: dict) -> bool:
    """True if the dict is non-empty and its timestamp is fresh."""
    if not hand_dict:
        return False
    ts = hand_dict.get("timestamp", 0.0)
    return (time.time() - ts) < HAND_TIMEOUT


# ── Scene ────────────────────────────────────────────────────────────────────
@configclass
class ArmHandSceneCfg(InteractiveSceneCfg):
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


# ── Main sim loop ─────────────────────────────────────────────────────────────
def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    robot = scene["robot"]
    robot_entity_cfg = SceneEntityCfg("robot", joint_names=[".*"], body_names=[".*"])
    robot_entity_cfg.resolve(scene)

    sim_dt = sim.get_physics_dt()

    # Initialise robot to default pose
    joint_pos = robot.data.default_joint_pos.clone()
    joint_vel = robot.data.default_joint_vel.clone()
    robot.write_joint_state_to_sim(joint_pos, joint_vel)

    # Build a mapping: joint_name → column index in the sim tensor
    sim_joint_names: list[str] = list(robot.data.joint_names)
    name_to_sim_idx: dict[str, int] = {name: i for i, name in enumerate(sim_joint_names)}

    # Initialize IK DexRetargeting Solver
    retargeter = None
    try:
        from dex_retargeting.retargeting_config import RetargetingConfig
        import numpy as np
        ik_config = {
            "type": "vector",
            "urdf_path": "/workspace/isaaclab/humanoid/autonomy/simulation/Humanoid_Wato/arm_assembly/arm_assembly_fixed.urdf",
            "wrist_link_name": "PALM_GAVIN_1DoF_Hinge_v2_1",
            "target_origin_link_names": [
                # Thumb (2 vectors)
                "CMC_THUMB_v1_1", "MCP_THUMB_v1_1",
                # Index (3 vectors: palm->MCP, MCP->PIP, PIP->DIP)
                "PALM_GAVIN_1DoF_Hinge_v2_1", "MCP_INDEX_v1_1", "PIP_INDEX_v1_1",
                # Middle (3 vectors)
                "PALM_GAVIN_1DoF_Hinge_v2_1", "MCP_MIDDLE_v1_1", "PIP_MIDDLE_v1_1",
                # Ring (3 vectors)
                "PALM_GAVIN_1DoF_Hinge_v2_1", "MCP_RING_v1_1", "PIP_RING_v1_1",
                # Pinky (3 vectors)
                "PALM_GAVIN_1DoF_Hinge_v2_1", "MCP_PINKY_v1_1", "PIP_PINKY_v1_1",
            ],
            "target_task_link_names": [
                # Thumb
                "MCP_THUMB_v1_1", "IP_THUMB_v1_1",
                # Index
                "MCP_INDEX_v1_1", "PIP_INDEX_v1_1", "DIP_INDEX_v1_1",
                # Middle
                "MCP_MIDDLE_v1_1", "PIP_MIDDLE_v1_1", "DIP_MIDDLE_v1_1",
                # Ring
                "MCP_RING_v1_1", "PIP_RING_v1_1", "DIP_RING_v1_1",
                # Pinky
                "MCP_PINKY_v1_1", "PIP_PINKY_v1_1", "DIP_PINKY_v1_1",
            ],
            "target_link_human_indices": np.array([
                # origins: [thumb_cmc, thumb_mcp, wrist, idx_mcp, idx_pip, wrist, mid_mcp, mid_pip, wrist, rng_mcp, rng_pip, wrist, pnk_mcp, pnk_pip]
                [1, 2,  0, 5, 6,  0, 9, 10,  0, 13, 14,  0, 17, 18],
                # targets: [thumb_mcp, thumb_ip, idx_mcp, idx_pip, idx_dip, mid_mcp, mid_pip, mid_dip, rng_mcp, rng_pip, rng_dip, pnk_mcp, pnk_pip, pnk_dip]
                [2, 3,  5, 6, 7,  9, 10, 11,  13, 14, 15,  17, 18, 19]
            ])
        }
        retargeter = RetargetingConfig.from_dict(ik_config).build()
        print(f"[INFO] Dex-Retargeting IK Solver active with {len(retargeter.joint_names)} joints.")
    except Exception as e:
        print(f"[WARNING] Could not load DexRetargeting IK Solver: {e}. Falling back to 1D joint mapping.")

    # ── Wait for first valid camera frame and snap robot to that pose ───────────
    print("[INFO]: Waiting for first hand frame from camera to set start pose...")
    while True:
        hand_dict = read_joint_file()
        if hand_dict:
            break
        # Tick the sim while waiting so it stays alive
        sim.step()
        scene.update(sim_dt)

    # Build initial joint target from the first observed frame
    joint_pos_target = robot.data.default_joint_pos.clone()
    for joint_name, angle in hand_dict.get("joints", {}).items():
        if joint_name in name_to_sim_idx:
            joint_pos_target[0, name_to_sim_idx[joint_name]] = float(angle)

    import json
    with open("/tmp/joint_limits.json", "w") as f:
        l = robot.data.soft_joint_pos_limits[0].cpu().numpy().tolist()
        json.dump({"names": robot.data.joint_names, "limits": l}, f)

    # Teleport the robot directly to the start pose (no physics convergence delay)
    robot.write_joint_state_to_sim(joint_pos_target, robot.data.default_joint_vel.clone())
    jd = hand_dict.get("joints", {})
    print(f"[INFO]: Start pose set from camera. forearm_rotation = "
          f"{jd.get('forearm_rotation', 0.0):.3f} rad, "
          f"wrist_extension = {jd.get('wrist_extension', 0.0):.3f} rad")

    # Slow EMA for arm joints that are contaminated by finger pose changes.
    # forearm_rotation is derived from pinky-to-index angle in image space,
    # which shifts during finger curling. A slow filter (alpha=0.04) damps
    # the finger-curl artifact while still tracking genuine wrist rotation.
    _ARM_SMOOTH_ALPHA = 0.15  # Moderate filter: responsive to wrist turns, damps finger-curl noise
    _arm_smoothed: dict[str, float] = {}

    hand_visible_prev = True

    while simulation_app.is_running():
        hand_dict = read_joint_file()
        hand_visible = is_hand_visible(hand_dict)

        # -- Normal tracking: update target from latest data --
        if hand_visible:
            # 1) Directly map arm / finger joints from 1D heuristic solver
            ARM_SLOW_JOINTS = {"forearm_rotation"}  # wrist_extension locked at 0 (see below)
            for joint_name, angle in hand_dict.get("joints", {}).items():
                if joint_name not in name_to_sim_idx:
                    continue
                raw = float(angle)
                if joint_name in ARM_SLOW_JOINTS:
                    # Route through a slow EMA to damp finger-curl contamination
                    prev = _arm_smoothed.get(joint_name, raw)
                    smoothed = _ARM_SMOOTH_ALPHA * raw + (1.0 - _ARM_SMOOTH_ALPHA) * prev
                    _arm_smoothed[joint_name] = smoothed
                    joint_pos_target[0, name_to_sim_idx[joint_name]] = smoothed
                else:
                    joint_pos_target[0, name_to_sim_idx[joint_name]] = raw

            # 2) Override fingers with real-time Cartesian IK if world data exists
            world_data = hand_dict.get("world", None)
            if world_data is not None and len(world_data) == 21 and retargeter is not None:
                import numpy as np
                if isinstance(world_data[0], dict):
                    world_np = np.array([[lm["x"], lm["y"], lm["z"]] for lm in world_data], dtype=np.float32)
                else:
                    world_np = np.array(world_data, dtype=np.float32)
                
                try:
                    # ── DYNAMIC FRAME ALIGNMENT ─────────────────────────────────────
                    # MediaPipe gives us coordinates relative to the camera's orientation.
                    # We MUST normalize the hand so the Palm always faces DOWN (Z) and Forward (Y).
                    wrist = world_np[0]
                    mid_mcp = world_np[9]
                    idx_mcp = world_np[5]
                    pnk_mcp = world_np[17]

                    # 1. Forward basis (Y): Wrist to Middle MCP
                    Y_h = mid_mcp - wrist
                    Y_h = Y_h / (np.linalg.norm(Y_h) + 1e-6)

                    # 2. Right basis (X): Cross logic. Right hand means Index is "Right" of Pinky
                    vec_pinky_idx = idx_mcp - pnk_mcp
                    Z_h = np.cross(vec_pinky_idx, Y_h) # Up/Back-of-hand vector
                    Z_h = Z_h / (np.linalg.norm(Z_h) + 1e-6)

                    X_h = np.cross(Y_h, Z_h)  # Right vector

                    # Transformation matrix (3x3). Columns are the orthogonal axes.
                    R_cam_to_hand = np.column_stack((X_h, Y_h, Z_h))

                    # ── FOREARM ROTATION FROM 3D PALM NORMAL ───────────────────────────
                    # Z_h is the back-of-hand vector in camera space — a full 3D signal
                    # that captures forearm roll across the complete range, unlike the
                    # 2D image-space kvec used by the 1D solver (which goes blind when
                    # the wrist is foreshortened from the camera's viewpoint).
                    # Roll = atan2 of Z_h projected in camera XZ plane.
                    forearm_3d = float(np.arctan2(Z_h[0], -Z_h[2]) + np.pi / 2)
                    forearm_3d = float(np.clip(forearm_3d, 0.0, np.pi))
                    if "forearm_rotation" in name_to_sim_idx:
                        prev_fr = _arm_smoothed.get("forearm_rotation", forearm_3d)
                        smoothed_fr = 0.15 * forearm_3d + 0.85 * prev_fr
                        _arm_smoothed["forearm_rotation"] = smoothed_fr
                        joint_pos_target[0, name_to_sim_idx["forearm_rotation"]] = smoothed_fr

                    # ── WRIST EXTENSION FROM 3D FINGER ELEVATION ─────────────────────
                    # Y_h = wrist→middle_MCP direction in camera space.
                    # MediaPipe Y-axis points DOWN, so -Y_h[1] = upward component.
                    # arcsin(-Y_h[1]) gives the elevation angle:
                    #   fingers up   → +1.57 rad
                    #   fingers level →  0.0  rad
                    #   fingers down → -1.57 rad
                    wrist_3d = float(np.arcsin(np.clip(-Y_h[1], -1.0, 1.0)))
                    wrist_3d = float(np.clip(wrist_3d, -1.57, 1.57))
                    if "wrist_extension" in name_to_sim_idx:
                        prev_we = _arm_smoothed.get("wrist_extension", wrist_3d)
                        smoothed_we = 0.25 * wrist_3d + 0.75 * prev_we
                        _arm_smoothed["wrist_extension"] = smoothed_we
                        joint_pos_target[0, name_to_sim_idx["wrist_extension"]] = smoothed_we
                    # ───────────────────────────────────────────────────────────────────

                    # Rotate all points into tracking-independent local basis
                    world_local = (world_np - wrist) @ R_cam_to_hand

                    # Align the target subset as angular direction vectors for the VectorOptimizer
                    # Layout: thumb(2) + each finger has 3 vectors: wrist->MCP, MCP->PIP, PIP->DIP
                    #   thumb:  cmc(1)->mcp(2), mcp(2)->ip(3)
                    #   index:  wrist(0)->mcp(5), mcp(5)->pip(6), pip(6)->dip(7)
                    #   middle: wrist(0)->mcp(9), mcp(9)->pip(10),pip(10)->dip(11)
                    #   ring:   wrist(0)->mcp(13),mcp(13)->pip(14),pip(14)->dip(15)
                    #   pinky:  wrist(0)->mcp(17),mcp(17)->pip(18),pip(18)->dip(19)
                    origin_indices = [1, 2,   0, 5, 6,   0, 9, 10,   0, 13, 14,   0, 17, 18]
                    target_indices = [2, 3,   5, 6, 7,   9, 10, 11,  13, 14, 15,  17, 18, 19]

                    target_points = world_local[target_indices]
                    origin_points = world_local[origin_indices]

                    # Compute directional vectors and normalize
                    target_vectors = target_points - origin_points
                    norms = np.linalg.norm(target_vectors, axis=1, keepdims=True)
                    target_vectors = target_vectors / (norms + 1e-6)


                    # VectorOptimizer takes the 5x3 direction array mapped natively 1:1
                    action = retargeter.retarget(target_vectors)
                    
                    # The solver spits out a 1D array perfectly ordered to retargeter.joint_names
                    for i, j_name in enumerate(retargeter.joint_names):
                        # Override hand joints; SKIP dip_ joints because DexRetargeting
                        # outputs 0.0 for them (no TIP link in URDF = no gradient).
                        # The 1D solver from wato_hand_ros2_node.py handles dip coupling.
                        if "dip_" in j_name:
                            continue
                        if any(finger in j_name for finger in ["thumb", "index", "middle", "ring", "pinky"]):
                            if j_name in name_to_sim_idx:
                                raw_angle = float(action[i])
                                joint_pos_target[0, name_to_sim_idx[j_name]] = raw_angle
                except Exception as e:
                    print(f"IK Solver Error: {e}")

            # --- DIP JOINT CORRECTION ---
            # DexRetargeting skips dip_ joints (no TIP link = no gradient).
            # Compute dip from IK-driven pip: biologically, dip ≈ 0.67 * pip (tendon coupling).
            # The URDF DIP links point in +Z at rest (not inline with finger), so we add
            # an offset that zeroes out when pip=0 (open) and adds curl as fingers close.
            import numpy as np
            for finger in ["index", "middle", "ring", "pinky"]:
                pip_name = f"pip_{finger}"
                dip_name = f"dip_{finger}"
                if pip_name in name_to_sim_idx and dip_name in name_to_sim_idx:
                    pip_val = float(joint_pos_target[0, name_to_sim_idx[pip_name]])
                    # Clamp pip to [0, pi] to compute a clean curl ratio
                    pip_curl = np.clip(pip_val / np.pi, 0.0, 1.0)
                    # URDF geometry offset: dip=0 is a 90° hook; offset needed to straighten:
                    # index/middle/ring/pinky DIP links all point in +Z at rest → need -1.57 offset
                    # to look straight. As finger curls (pip_curl→1), we move toward 0 (touching palm).
                    dip_val = -1.57 * (1.0 - pip_curl) + 0.7 * pip_val
                    joint_pos_target[0, name_to_sim_idx[dip_name]] = float(dip_val)

        hand_visible_prev = hand_visible

        # -- Apply to robot --
        joint_pos_des = joint_pos_target[:, robot_entity_cfg.joint_ids].clone()
        robot.set_joint_position_target(joint_pos_des, joint_ids=robot_entity_cfg.joint_ids)
        scene.write_data_to_sim()

        sim.step()
        scene.update(sim_dt)


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])

    scene_cfg = ArmHandSceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    sim.reset()

    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
