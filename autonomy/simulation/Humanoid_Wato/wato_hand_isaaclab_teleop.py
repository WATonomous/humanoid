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
                "CMC_THUMB_v1_1", "MCP_THUMB_v1_1",
                "MCP_INDEX_v1_1", "PIP_INDEX_v1_1",
                "MCP_MIDDLE_v1_1", "PIP_MIDDLE_v1_1",
                "MCP_RING_v1_1", "PIP_RING_v1_1",
                "MCP_PINKY_v1_1", "PIP_PINKY_v1_1"
            ],
            "target_task_link_names": [
                "MCP_THUMB_v1_1", "IP_THUMB_v1_1",
                "PIP_INDEX_v1_1", "DIP_INDEX_v1_1",
                "PIP_MIDDLE_v1_1", "DIP_MIDDLE_v1_1",
                "PIP_RING_v1_1", "DIP_RING_v1_1",
                "PIP_PINKY_v1_1", "DIP_PINKY_v1_1"
            ],
            "target_link_human_indices": np.array([
                [1, 2, 5, 6, 9, 10, 13, 14, 17, 18],
                [2, 3, 6, 7, 10, 11, 14, 15, 18, 19]
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

    hand_visible_prev = True

    while simulation_app.is_running():
        hand_dict = read_joint_file()
        hand_visible = is_hand_visible(hand_dict)

        # -- Normal tracking: update target from latest data --
        if hand_visible:
            # 1) Directly map arm joints / safe joints
            for joint_name, angle in hand_dict.get("joints", {}).items():
                if joint_name in name_to_sim_idx:
                    joint_pos_target[0, name_to_sim_idx[joint_name]] = float(angle)

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

                    # Rotate all points into tracking-independent local basis
                    world_local = (world_np - wrist) @ R_cam_to_hand

                    # Align the target subset as angular direction vectors for the VectorOptimizer
                    # We pair 2 vectors per finger: Proximal phalanx (MCP to PIP) and Middle phalanx (PIP to DIP)
                    target_indices = [2, 3, 6, 7, 10, 11, 14, 15, 18, 19]
                    origin_indices = [1, 2, 5, 6, 9, 10, 13, 14, 17, 18]
                    
                    target_points = world_local[target_indices]
                    origin_points = world_local[origin_indices]
                    
                    # Compute directional vectors and normalize them uniformly
                    target_vectors = target_points - origin_points
                    norms = np.linalg.norm(target_vectors, axis=1, keepdims=True)
                    target_vectors = target_vectors / (norms + 1e-6)


                    # VectorOptimizer takes the 5x3 direction array mapped natively 1:1
                    action = retargeter.retarget(target_vectors)
                    
                    # The solver spits out a 1D array perfectly ordered to retargeter.joint_names
                    for i, j_name in enumerate(retargeter.joint_names):
                        # Only override if it's a hand joint (we let arm tracking stay manual)
                        if any(finger in j_name for finger in ["thumb", "index", "middle", "ring", "pinky"]):
                            if j_name in name_to_sim_idx:
                                raw_angle = float(action[i])
                                joint_pos_target[0, name_to_sim_idx[j_name]] = raw_angle
                except Exception as e:
                    print(f"IK Solver Error: {e}")

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
