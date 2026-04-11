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

# Force enable cameras — required for camera sensors to initialize
if not hasattr(args_cli, 'enable_cameras') or not args_cli.enable_cameras:
    args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ── Now safe to import Isaac / torch ─────────────────────────────────────────
import torch                                        # noqa: E402

import isaaclab.sim as sim_utils                   # noqa: E402
from isaaclab.assets import AssetBaseCfg, ArticulationCfg
from isaaclab.managers import SceneEntityCfg       # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils import configclass             # noqa: E402

from HumanoidRL.HumanoidRLPackage.HumanoidRLSetup.modelCfg.humanoid import ARM_CFG  # noqa: E402
from isaaclab.assets import RigidObjectCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.sensors import Camera, CameraCfg
from isaaclab.utils.math import quat_from_euler_xyz

import numpy as np
import h5py
import re
from pathlib import Path
from datetime import datetime

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("[WARN] OpenCV not available; video recording will use imageio")

try:
    import imageio
    IMAGEIO_AVAILABLE = True
except ImportError:
    IMAGEIO_AVAILABLE = False

try:
    import omni.ui as ui
    OMNI_UI_AVAILABLE = True
except ImportError:
    OMNI_UI_AVAILABLE = False

def quat_to_matrix(q):
    """
    Convert quaternion (w, x, y, z) → 3x3 rotation matrix
    q: tensor of shape (4,)
    """
    w, x, y, z = q

    return torch.tensor([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ], device=q.device)


def _get_cam_quat(azimuth_deg: float, pitch_deg: float = 25.0):
    """Quaternion (w,x,y,z) for a camera at azimuth_deg looking toward origin, tilted down by pitch_deg."""
    try:
        az = float(np.deg2rad(azimuth_deg))
        pt = float(np.deg2rad(pitch_deg))
        q = quat_from_euler_xyz(
            torch.tensor([0.0]),
            torch.tensor([pt]),
            torch.tensor([az + np.pi]),
        )
        return tuple(q[0].tolist())
    except Exception:
        return (1.0, 0.0, 0.0, 0.0)


class DemonstrationRecorder:
    """
    Records robot hand demonstrations for Imitation Learning.

    HDF5 per-episode schema (demonstrations/robot_demos_N.hdf5)
    -------------------------------------------------------------
    episode_N/
        observations/
            finger_joint_pos   (T, N_f)   all finger DOF positions
            finger_joint_vel   (T, N_f)   all finger DOF velocities
            arm_joint_pos      (T, N_a)   shoulder + elbow positions
            arm_joint_vel      (T, N_a)   shoulder + elbow velocities
            wrist_state        (T, 2)     [forearm_rotation, wrist_extension]
            palm_pose          (T, 7)     palm pos(3) + quat(4) in world frame
            door_hinge_angle   (T, 1)     door joint angle
            hand_visible       (T, 1)     MediaPipe tracking quality flag
        actions/
            finger_targets     (T, N_f)
            arm_targets        (T, N_a)
            wrist_targets      (T, 2)
        attrs: video_path_<cam> for each saved MP4

    MP4 videos saved to: recordings/episode_N_<cam>.mp4
    """

    def __init__(self, save_dir: str = "demonstrations", recordings_dir: str = "recordings"):
        self.save_dir = Path(save_dir)
        self.save_dir.mkdir(exist_ok=True, parents=True)
        self.recordings_dir = Path(recordings_dir)
        self.recordings_dir.mkdir(exist_ok=True, parents=True)
        self.episodes: list = []
        self.recording = False
        self.video_fps = 10

        # Resume episode counter from existing recordings
        self.episode_counter = 0
        pattern = re.compile(r"episode_(\d+)_")
        max_ep = -1
        for fp in self.recordings_dir.glob("*.mp4"):
            m = pattern.search(fp.name)
            if m:
                try:
                    ep_num = int(m.group(1))
                    if ep_num > max_ep:
                        max_ep = ep_num
                except ValueError:
                    pass
        if max_ep >= 0:
            self.episode_counter = max_ep + 1
            print(f"[INFO] Resuming from episode_{self.episode_counter} (found up to episode_{max_ep})")

        self._current: dict = {}
        print(f"[INFO] DemonstrationRecorder ready  →  {self.save_dir.absolute()}")

    def _blank_episode(self) -> dict:
        return {
            "episode_num": self.episode_counter,
            "obs": {
                "finger_joint_pos": [], "finger_joint_vel": [],
                "arm_joint_pos":    [], "arm_joint_vel":    [],
                "wrist_state":      [],
                "palm_pose":        [],
                "door_hinge_angle": [],
                "hand_visible":     [],
            },
            "actions": {
                "finger_targets": [],
                "arm_targets":    [],
                "wrist_targets":  [],
            },
            "video_frames": {},
        }

    def start_episode(self):
        """Begin a new recording episode."""
        if self.recording:
            print("[WARN] Already recording — stop the current episode first")
            return
        self.recording = True
        self._current = self._blank_episode()
        print(f"[RECORDING] ▶  Started episode_{self.episode_counter}")

    def add_transition(self, obs_dict: dict, action_dict: dict, video_frames_dict: dict = None):
        """Append one timestep of data."""
        if not self.recording:
            return

        def _to_np(t):
            if t is None:
                return np.zeros(1)
            if hasattr(t, "cpu"):
                return t.squeeze(0).cpu().numpy()
            return np.asarray(t)

        for k, v in obs_dict.items():
            if k in self._current["obs"]:
                self._current["obs"][k].append(_to_np(v))

        for k, v in action_dict.items():
            if k in self._current["actions"]:
                self._current["actions"][k].append(_to_np(v))

        if video_frames_dict:
            for cam, frame in video_frames_dict.items():
                if cam not in self._current["video_frames"]:
                    self._current["video_frames"][cam] = []
                if hasattr(frame, "cpu"):
                    frame = frame.cpu().numpy()
                frame = np.asarray(frame)
                if frame.dtype != np.uint8:
                    frame = (np.clip(frame * 255, 0, 255) if frame.max() <= 1.0
                             else np.clip(frame, 0, 255)).astype(np.uint8)
                self._current["video_frames"][cam].append(frame)

    def _save_video(self, frames: list, episode_num: int, cam_name: str):
        if not frames:
            return None
        video_path = self.recordings_dir / f"episode_{episode_num}_{cam_name}.mp4"
        try:
            if CV2_AVAILABLE:
                h, w = frames[0].shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                out = cv2.VideoWriter(str(video_path), fourcc, self.video_fps, (w, h))
                for f in frames:
                    bgr = cv2.cvtColor(f.astype(np.uint8), cv2.COLOR_RGB2BGR) if f.ndim == 3 and f.shape[2] == 3 else f.astype(np.uint8)
                    out.write(bgr)
                out.release()
            elif IMAGEIO_AVAILABLE:
                imageio.mimwrite(str(video_path), [f.astype(np.uint8) for f in frames], fps=self.video_fps)
            else:
                print("[WARN] No video library (cv2/imageio). Video not saved.")
                return None
            print(f"[VIDEO] {len(frames)} frames → {video_path}")
            return str(video_path)
        except Exception as e:
            print(f"[ERROR] Video save failed ({cam_name}): {e}")
            return None

    def end_episode(self):
        """Stop recording, save MP4s and HDF5."""
        if not self.recording:
            print("[WARN] Not currently recording")
            return
        self.recording = False
        n_steps = len(self._current["obs"].get("finger_joint_pos", []))
        if n_steps == 0:
            print("[WARN] Episode ended with no data — discarding")
            return
        ep_num = self._current["episode_num"]
        # Save MP4 per camera
        video_paths = {}
        for cam, frames in self._current["video_frames"].items():
            path = self._save_video(frames, ep_num, cam)
            if path:
                video_paths[cam] = path
        self._current["video_paths"] = video_paths
        self._current["video_frames"] = {}  # free memory
        self.episodes.append(self._current)
        self.episode_counter += 1
        print(f"[RECORDING] ⏹  Episode {ep_num}: {n_steps} steps, {len(video_paths)} videos")
        self.save()

    def save(self):
        """Flush all in-memory episodes to individual HDF5 files."""
        if not self.episodes:
            return
        for ep in self.episodes:
            ep_num = ep["episode_num"]
            hdf5_path = self.save_dir / f"robot_demos_{ep_num}.hdf5"
            try:
                with h5py.File(hdf5_path, "w") as f:
                    grp = f.create_group(f"episode_{ep_num}")
                    obs_grp = grp.create_group("observations")
                    for k, v in ep["obs"].items():
                        if v:
                            obs_grp.create_dataset(k, data=np.array(v))
                    act_grp = grp.create_group("actions")
                    for k, v in ep["actions"].items():
                        if v:
                            act_grp.create_dataset(k, data=np.array(v))
                    for cam, path in ep.get("video_paths", {}).items():
                        grp.attrs[f"video_path_{cam}"] = path
                print(f"[SUCCESS] HDF5 → {hdf5_path.absolute()}")
            except Exception as e:
                print(f"[ERROR] HDF5 save failed: {e}")
        self.episodes.clear()


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

    door = ArticulationCfg(
        prim_path="/World/Door",
        spawn=sim_utils.UrdfFileCfg(
            asset_path="/workspace/isaaclab/humanoid/autonomy/simulation/Humanoid_Wato/arm_assembly/simple_door.urdf",
            fix_base=True,
            scale=(0.3, 0.5, 0.5),
            joint_drive=sim_utils.UrdfFileCfg.JointDriveCfg(
                drive_type="force",
                target_type="position",
                gains=sim_utils.UrdfFileCfg.JointDriveCfg.PDGainsCfg(
                    stiffness=100.0,
                    damping=10.0,
                ),
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0, 0.5, -0.25), 
            rot=(0.707, 0.0, 0.0, 0.707),
            joint_pos={"door_hinge": 0.0},
        ),
        actuators={
            "hinge": ImplicitActuatorCfg(
                joint_names_expr=["door_hinge"],
                effort_limit=50.0, velocity_limit=1.0,
                stiffness=100.0,
                damping=10.0,
            ),
        },
    )

    # ── 9 Cameras for Imitation Learning data collection ────────────────────
    # Camera 1: Front  (0°, h=0.8m, r=1.2m)
    camera_front = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Camera_front",
        update_period=0.1, height=480, width=640, data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0,
            horizontal_aperture=20.955, clipping_range=(0.1, 10.0),
        ),
        offset=CameraCfg.OffsetCfg(pos=(1.2, 0.0, 0.8), rot=_get_cam_quat(0.0, 25.0), convention="world"),
    )
    # Camera 2: Front High  (0°, h=1.5m, r=0.8m)
    camera_front_high = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Camera_front_high",
        update_period=0.1, height=480, width=640, data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0,
            horizontal_aperture=20.955, clipping_range=(0.1, 10.0),
        ),
        offset=CameraCfg.OffsetCfg(pos=(0.8, 0.0, 1.5), rot=_get_cam_quat(0.0, 55.0), convention="world"),
    )
    # Camera 3: Left  (90°, h=0.8m, r=1.2m)
    camera_left = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Camera_left",
        update_period=0.1, height=480, width=640, data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0,
            horizontal_aperture=20.955, clipping_range=(0.1, 10.0),
        ),
        offset=CameraCfg.OffsetCfg(pos=(0.0, 1.2, 0.8), rot=_get_cam_quat(90.0, 25.0), convention="world"),
    )
    # Camera 4: Right  (270°, h=0.8m, r=1.2m)
    camera_right = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Camera_right",
        update_period=0.1, height=480, width=640, data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0,
            horizontal_aperture=20.955, clipping_range=(0.1, 10.0),
        ),
        offset=CameraCfg.OffsetCfg(pos=(0.0, -1.2, 0.8), rot=_get_cam_quat(270.0, 25.0), convention="world"),
    )
    # Camera 5: Back  (180°, h=0.8m, r=1.2m)
    camera_back = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Camera_back",
        update_period=0.1, height=480, width=640, data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0,
            horizontal_aperture=20.955, clipping_range=(0.1, 10.0),
        ),
        offset=CameraCfg.OffsetCfg(pos=(-1.2, 0.0, 0.8), rot=_get_cam_quat(180.0, 25.0), convention="world"),
    )
    # Camera 6: Front Low  (0°, h=0.3m) — looks slightly upward at fingers
    camera_front_low = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Camera_front_low",
        update_period=0.1, height=480, width=640, data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0,
            horizontal_aperture=20.955, clipping_range=(0.1, 10.0),
        ),
        offset=CameraCfg.OffsetCfg(pos=(0.8, 0.0, 0.3), rot=_get_cam_quat(0.0, -10.0), convention="world"),
    )
    # Camera 7: Diagonal Left  (45°, h=1.0m)
    camera_diag_left = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Camera_diag_left",
        update_period=0.1, height=480, width=640, data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0,
            horizontal_aperture=20.955, clipping_range=(0.1, 10.0),
        ),
        offset=CameraCfg.OffsetCfg(pos=(0.85, 0.85, 1.0), rot=_get_cam_quat(45.0, 30.0), convention="world"),
    )
    # Camera 8: Diagonal Right  (315°, h=1.0m)
    camera_diag_right = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Camera_diag_right",
        update_period=0.1, height=480, width=640, data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0,
            horizontal_aperture=20.955, clipping_range=(0.1, 10.0),
        ),
        offset=CameraCfg.OffsetCfg(pos=(0.85, -0.85, 1.0), rot=_get_cam_quat(315.0, 30.0), convention="world"),
    )
    # Camera 9: Wrist-mounted — attached to palm link, moves with the robot
    # Positioned above/forward of palm looking down at the fingers
    camera_wrist = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/PALM_GAVIN_1DoF_Hinge_v2_1/wrist_cam",
        update_period=0.1, height=240, width=320, data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=12.0, focus_distance=0.3,
            horizontal_aperture=20.955, clipping_range=(0.01, 3.0),
        ),
        offset=CameraCfg.OffsetCfg(
            pos=(0.0, 0.05, 0.1),
            rot=(0.7071, 0.0, 0.7071, 0.0),
            convention="ros",
        ),
    )


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

    # ── Precompute joint index groups for IL recording (done once) ──────────────
    _jn_lower = [n.lower() for n in sim_joint_names]
    _FINGER_KW = ("thumb", "index", "middle", "ring", "pinky", "mcp_", "pip_", "dip_", "ip_", "cmc_")
    _ARM_KW    = ("shoulder_flexion_extension", "shoulder_rotation", "elbow_flexion_extension")
    _WRIST_KW  = ("forearm_rotation", "wrist_extension")
    _finger_ids = [i for i, n in enumerate(_jn_lower) if any(k in n for k in _FINGER_KW)]
    _arm_ids    = [name_to_sim_idx[k] for k in _ARM_KW    if k in name_to_sim_idx]
    _wrist_ids  = [name_to_sim_idx[k] for k in _WRIST_KW  if k in name_to_sim_idx]
    _palm_body_idx = robot.data.body_names.index("PALM_GAVIN_1DoF_Hinge_v2_1")
    # Pre-look up door for recording
    _door_obj   = scene["door"]
    _door_jnames = list(_door_obj.data.joint_names)
    _door_ridx  = _door_jnames.index("door_hinge")
    print(f"[INFO] IL groups — fingers: {len(_finger_ids)} DOF, arm: {len(_arm_ids)} DOF, wrist: {len(_wrist_ids)} DOF")

    # ── Demonstration Recorder ─────────────────────────────────────────────────
    recorder = DemonstrationRecorder("demonstrations", "recordings")

    # ── UI Recording Buttons (GUI mode only) ───────────────────────────────────
    if OMNI_UI_AVAILABLE and not args_cli.headless:
        _rec_window = ui.Window("IL Recording", width=230, height=140)
        with _rec_window.frame:
            with ui.VStack(spacing=8):
                ui.Label("Demonstration Recording", height=22)
                ui.Button("▶  Start Recording",  clicked_fn=recorder.start_episode, height=44)
                ui.Button("⏹  Stop & Save",       clicked_fn=recorder.end_episode,   height=44)
        print("[INFO] Recording UI window created")
    else:
        print("[INFO] Headless mode: call recorder.start_episode() / recorder.end_episode() programmatically")
    # ───────────────────────────────────────────────────────────────────────

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

    # ── ARM POSITION TRACKING CONFIG ────────────────────────────────────────────
    # Maps 2D wrist position and hand scale (depth proxy) to shoulder/elbow joints.
    # GAINS: radians per normalized unit.  Tune if arm motion feels too big/small.
    ARM_SHOULDER_FE_GAIN  =  2.0   # Blue  arrow: up/down (raised — was 0.68)
    ARM_SHOULDER_AA_GAIN  =  1.8   # Green arrow: sideways (1.5×)
    ARM_ELBOW_FE_GAIN     =  0.7   # Red   arrow: forward via elbow extension
    ARM_POS_CALIB_FRAMES =  30    # Frames to average for the neutral reference
    ARM_POS_ALPHA        =  0.06  # EMA for active axis: silky smooth motion
    ARM_RETURN_ALPHA     =  0.06  # EMA for inactive axes: fast snap back to neutral
    ARM_DEADZONE_XY       =  0.02  # Height/sideways: dead band around screen center
    ARM_DEADZONE_SIDEWAYS =  0.06  # Sideways-specific: wider (screen-X noisier)
    ARM_DEADZONE_SCALE    =  0.22  # Forward: ignore depth changes < 8% of neutral scale
    # Fixed neutral depth reference: wrist→middle-MCP distance when hand is at working distance.
    # Increase if your hand appears SMALL at neutral; decrease if it appears LARGE.
    # Purely absolute — no per-session calibration needed.
    ARM_NEUTRAL_SCALE     =  0.20  # ~20% of frame height = neutral working distance
    # Per-joint clamps [min, max] in radians.  min=0 prevents backward bending.
    ARM_JOINT_CLAMPS = {
        "elbow_flexion_extension":    (-1.0,  1.0),  # was (-1.0, 0.6)
        "shoulder_flexion_extension": (-1,  1),
        "shoulder_rotation":          (-0.7,  0.7),
    }
    _arm_pos_ref: dict | None = None
    _arm_pos_count: int = 0
    # Pre-initialize at 0 so calibration phase holds joints at neutral with no snap
    _arm_pos_smoothed: dict[str, float] = {
        "elbow_flexion_extension":      0.0,
        "shoulder_flexion_extension":   0.0,
        "shoulder_rotation":            0.0,
    }
    # ────────────────────────────────────────────────────────────────────────────

    hand_visible_prev = True

    while simulation_app.is_running():
        hand_dict = read_joint_file()
        hand_visible = is_hand_visible(hand_dict)
        world_local = None  # reset each frame
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

            # 2) ARM POSITION TRACKING from 2D image landmarks ──────────────────
            # Blue  (up/down)   → shoulder_flexion_extension
            # Green (sideways)  → shoulder_abduction_adduction
            # Red   (forward)   → elbow_flexion_extension  (hand scale = depth proxy)
            import math as _math
            lm_data = hand_dict.get("landmarks", None)
            if lm_data and len(lm_data) >= 21:
                # lm_data[0] = WRIST = bottom of palm. All position tracking anchors here.
                wx  = float(lm_data[0]["x"])   # 0=left, 1=right
                wy  = float(lm_data[0]["y"])   # 0=top,  1=bottom  (MediaPipe Y↓)

                # Depth signal: WRIST (lm[0]) → MIDDLE MCP (lm[9]) distance in image.
                # Uses bottom-of-palm as anchor. Grows when hand is closer to camera.
                smx   = float(lm_data[9]["x"]) - wx
                smy   = float(lm_data[9]["y"]) - wy
                scale = _math.sqrt(smx*smx + smy*smy)

                # Startup hold: lock arm for first ARM_POS_CALIB_FRAMES frames so the
                # robot doesn't jitter before tracking begins. No reference captured.
                if _arm_pos_count < ARM_POS_CALIB_FRAMES:
                    _arm_pos_count += 1
                    for jname in _arm_pos_smoothed:
                        if jname in name_to_sim_idx:
                            joint_pos_target[0, name_to_sim_idx[jname]] = 0.0
                else:
                    # FULLY ABSOLUTE depth: compare live scale to fixed neutral constant.
                    # d_scl > 1 = hand is closer than neutral  (arm should extend)
                    # d_scl < 1 = hand is farther than neutral (arm returns to 0)
                    # ── SIGNAL SMOOTHING PIPELINE ────────────────────────────────────
                    # Step 1: Median filter on raw scale to kill noise spikes
                    if not hasattr(run_simulator, '_scale_buf'):
                        run_simulator._scale_buf = []
                    run_simulator._scale_buf.append(scale)
                    if len(run_simulator._scale_buf) > 7:   # 7-frame median window
                        run_simulator._scale_buf.pop(0)
                    scale_filtered = sorted(run_simulator._scale_buf)[len(run_simulator._scale_buf) // 2]

                    d_scl = scale_filtered / max(ARM_NEUTRAL_SCALE, 1e-4)
                    d_scl = d_scl if abs(d_scl - 1.0) > ARM_DEADZONE_SCALE else 1.0

                    height_abs   = (0.5 - wy) * 2.0
                    height_abs   = height_abs if abs(height_abs) > ARM_DEADZONE_XY * 2 else 0.0

                    sideways_abs = (wx - 0.5) * 2.0
                    sideways_abs = sideways_abs if abs(sideways_abs) > ARM_DEADZONE_SIDEWAYS else 0.0

                    forward_target = (d_scl - 1.0)

                    forward_active = forward_target != 0.0
                    height_active  = height_abs    != 0.0
                    side_active    = sideways_abs  != 0.0

                    arm_targets = {
                        "elbow_flexion_extension":    (0.3 * ARM_SHOULDER_FE_GAIN * height_abs if height_active else 0.0)
                                                    - 1.2 * _arm_pos_smoothed.get("shoulder_flexion_extension", 0.0),
                        "shoulder_flexion_extension": -0.5*ARM_ELBOW_FE_GAIN * forward_target if forward_active else 0.0,
                        "shoulder_rotation":           -0.8*ARM_SHOULDER_AA_GAIN * sideways_abs if side_active    else 0.0,
                    }

                    ARM_MAX_DELTA = 0.012   # max rad/frame — tune down to slow further
                    ARM_MAX_ACCEL = 0.004   # max change in delta — kills jerk

                    for jname, jval in arm_targets.items():
                        if jname not in name_to_sim_idx:
                            continue
                        prev_val  = _arm_pos_smoothed.get(jname, 0.0)
                        prev_delta = _arm_pos_smoothed.get(jname + "_delta", 0.0)

                        alpha = ARM_POS_ALPHA if jval != 0.0 else ARM_RETURN_ALPHA
                        raw_smooth = alpha * jval + (1.0 - alpha) * prev_val

                        # Velocity clamp
                        delta = raw_smooth - prev_val
                        delta = max(-ARM_MAX_DELTA, min(ARM_MAX_DELTA, delta))

                        # Acceleration clamp (jerk limiter)
                        delta_change = delta - prev_delta
                        delta_change = max(-ARM_MAX_ACCEL, min(ARM_MAX_ACCEL, delta_change))
                        delta = prev_delta + delta_change

                        smoothed_val = prev_val + delta
                        lo, hi = ARM_JOINT_CLAMPS.get(jname, (-3.14, 3.14))
                        smoothed_val = max(lo, min(hi, smoothed_val))

                        _arm_pos_smoothed[jname] = smoothed_val
                        _arm_pos_smoothed[jname + "_delta"] = delta
                        joint_pos_target[0, name_to_sim_idx[jname]] = smoothed_val
# ─────────────────────────────────────────────────────────────────
            # ────────────────────────────────────────────────────────────────────

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

                    # ── WRIST ORIENTATION: SPHERICAL DECOMPOSITION OF Z_h ──────────────
                    # Z_h = back-of-hand unit vector in camera space.
                    # We decompose it into two independent angular components:
                    #   azimuth  → forearm_rotation (palm rolling left/right)
                    #   elevation→ wrist_extension  (palm tilting up/down)
                    # Using the SAME vector for both joints prevents kinematic coupling
                    # (the diagonal artifact that appears when two different signals
                    # are fed into two series-connected joints independently).

                    # Azimuth: project Z_h onto horizontal plane, find angle
                    zh_horiz = np.array([Z_h[0], 0.0, Z_h[2]], dtype=np.float64)
                    zh_len   = float(np.linalg.norm(zh_horiz))
                    if zh_len > 0.1:
                        zh_hat      = zh_horiz / zh_len
                        forearm_3d  = float(np.arctan2(zh_hat[0], -zh_hat[2]) + np.pi / 2)
                    else:
                        forearm_3d  = _arm_smoothed.get("forearm_rotation", np.pi / 2)
                    forearm_3d = float(np.clip(forearm_3d, 0.0, np.pi))

                    # Elevation: signed angle of Z_h above/below horizontal
                    # Z_h[1] = 0 → palm vertical (camera-facing) → wrist neutral = 0
                    # Z_h[1] < 0 → back-of-hand tilts down  → palm tilts up
                    # Z_h[1] > 0 → back-of-hand tilts up    → palm tilts down
                    wrist_3d = float(np.arcsin(np.clip(-Z_h[1], -1.0, 1.0)))
                    wrist_3d = float(np.clip(wrist_3d, -1.4, 1.4))

                    # Permanent offset applied AFTER EMA so the clip on forearm_3d
                    # above never eats it. Isaac Lab joint limits handle range enforcement.
                    FOREARM_ROT_OFFSET = np.pi / 2   # 90° — tune this value as needed

                    if "forearm_rotation" in name_to_sim_idx:
                        prev_fr    = _arm_smoothed.get("forearm_rotation", forearm_3d)
                        smoothed_fr = 0.15 * forearm_3d + 0.85 * prev_fr
                        _arm_smoothed["forearm_rotation"] = smoothed_fr
                        joint_pos_target[0, name_to_sim_idx["forearm_rotation"]] = smoothed_fr + FOREARM_ROT_OFFSET

                    if "wrist_extension" in name_to_sim_idx:
                        prev_we    = _arm_smoothed.get("wrist_extension", wrist_3d)
                        smoothed_we = 0.20 * wrist_3d + 0.80 * prev_we
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

        # -- IL DATA COLLECTION (every step; only stores while recorder.recording) --
        def _get_cam_frame(sensor):
            try:
                rgb = sensor.data.output.get("rgb")
                if rgb is None:
                    return None
                frame = rgb[0] if rgb.ndim == 4 else rgb
                if hasattr(frame, "cpu"):
                    frame = frame.cpu().numpy()
                else:
                    frame = np.asarray(frame)
                if frame.dtype != np.uint8:
                    fmax = frame.max()
                    frame = np.clip(frame * 255, 0, 255).astype(np.uint8) if fmax <= 1.0 else np.clip(frame, 0, 255).astype(np.uint8)
                return frame
            except Exception as _e:
                if not hasattr(sensor, "_frame_err"):
                    print(f"[WARN] Frame capture: {_e}")
                    sensor._frame_err = True
                return None
        _palm_pos_il  = robot.data.body_pos_w[:, _palm_body_idx, :]
        _palm_quat_il = robot.data.body_quat_w[:, _palm_body_idx, :]
        _palm_pose_il = torch.cat([_palm_pos_il, _palm_quat_il], dim=-1)
        _door_angle_il = _door_obj.data.joint_pos[:, _door_ridx:_door_ridx + 1]
        _obs_dict = {
            "finger_joint_pos": robot.data.joint_pos[:, _finger_ids] if _finger_ids else torch.zeros(1, 1, device=sim.device),
            "finger_joint_vel": robot.data.joint_vel[:, _finger_ids] if _finger_ids else torch.zeros(1, 1, device=sim.device),
            "arm_joint_pos":    robot.data.joint_pos[:, _arm_ids]    if _arm_ids    else torch.zeros(1, 3, device=sim.device),
            "arm_joint_vel":    robot.data.joint_vel[:, _arm_ids]    if _arm_ids    else torch.zeros(1, 3, device=sim.device),
            "wrist_state":      robot.data.joint_pos[:, _wrist_ids]  if _wrist_ids  else torch.zeros(1, 2, device=sim.device),
            "palm_pose":        _palm_pose_il,
            "door_hinge_angle": _door_angle_il,
            "hand_visible":     torch.tensor([[1.0 if hand_visible else 0.0]], device=sim.device),
        }
        _action_dict = {
            "finger_targets": joint_pos_target[:, _finger_ids] if _finger_ids else torch.zeros(1, 1, device=sim.device),
            "arm_targets":    joint_pos_target[:, _arm_ids]    if _arm_ids    else torch.zeros(1, 3, device=sim.device),
            "wrist_targets":  joint_pos_target[:, _wrist_ids]  if _wrist_ids  else torch.zeros(1, 2, device=sim.device),
        }
        _vid_dict = {}
        for _ck in scene.keys():
            if "camera" in _ck.lower():
                try:
                    _cs = scene[_ck]
                    if hasattr(_cs, "data") and hasattr(_cs.data, "output"):
                        _cf = _get_cam_frame(_cs)
                        if _cf is not None:
                            _vid_dict[_ck] = _cf
                except Exception:
                    pass
        recorder.add_transition(_obs_dict, _action_dict, _vid_dict)

       # --- DOOR PUSH / PULL (PURE SPATIAL) ---

        door_obj = scene["door"]
        door_joint_names = list(door_obj.data.joint_names)
        door_idx = door_joint_names.index("door_hinge")

        palm_body_idx = robot.data.body_names.index("PALM_GAVIN_1DoF_Hinge_v2_1")
        palm_pos = robot.data.body_pos_w[0, palm_body_idx]

        panel_idx = door_obj.data.body_names.index("door_panel")
        panel_pos = door_obj.data.body_pos_w[0, panel_idx]

        # Get door orientation
        panel_quat = door_obj.data.body_quat_w[0, panel_idx]
        print("quat:", panel_quat)
        R = quat_to_matrix(panel_quat)

        # Door normal (TRY AXIS if needed)
        door_normal = R[:, 0]  # try 1 or 2 if wrong

        # Perpendicular distance to door surface
        vec = palm_pos - panel_pos
        dist_to_plane = float(torch.dot(vec, door_normal))
        surface_dist = abs(dist_to_plane)

        print(f"[SURFACE] dist={surface_dist:.3f}")
        SURFACE_THRESH = 0.14  # 14 cm
        is_touching_surface = surface_dist < SURFACE_THRESH
        is_in_front = dist_to_plane > 0

        hinge_pos = door_obj.data.body_pos_w[0, 0]  # hinge body

        # Distances
        dist_to_panel = float(torch.norm(palm_pos[:2] - panel_pos[:2]))
        dist_to_hinge = float(torch.norm(palm_pos[:2] - hinge_pos[:2]))

        # Motion (velocity)
        prev_palm_x = getattr(run_simulator, '_prev_palm_x', float(palm_pos[0]))
        palm_dx = float(palm_pos[0]) - prev_palm_x
        run_simulator._prev_palm_x = float(palm_pos[0])

        # Door state
        current_door = float(door_obj.data.joint_pos[0, door_idx])

        # Moment arm (avoid divide-by-zero)
        hinge_to_palm = max(float(torch.norm(palm_pos[:2] - hinge_pos[:2])), 0.05)

        # Thresholds
        PANEL_THRESH = 0.25
        HINGE_THRESH = 0.15

        target = torch.zeros(1, len(door_joint_names), device=palm_pos.device)

        # -------------------------------
        # PUSH: near panel → push forward
        # -------------------------------
        print(f"[PUSH] dist_panel={dist_to_panel:.3f} | dx={palm_dx:.4f}")
        print(f"[PULL] dist_hinge={dist_to_hinge:.3f} | dx={palm_dx:.4f}")

        PROXIMITY_THRESHOLD = 0.3  # must be within 30cm of the panel to interact
        if not (is_touching_surface):
            print(f"[DEBUG] Not touching door surface ({surface_dist:.3f}m), dist_to_panel={dist_to_panel:.3f}, dist_to_hinge={dist_to_hinge:.3f}")
            # --- HARD STOP: no interaction ---
            effort = torch.zeros(1, len(door_joint_names), device=palm_pos.device)
            door_obj.set_joint_effort_target(effort)

            # Also remove any external forces on the hand
            num_bodies = robot.data.body_pos_w.shape[1]
            forces = torch.zeros((1, num_bodies, 3), device=palm_pos.device)
            torques = torch.zeros((1, num_bodies, 3), device=palm_pos.device)

            robot.set_external_force_and_torque(forces=forces, torques=torques)

            continue  # skip rest of logic
        else:
            print(f"[DEBUG] Touching door surface ({surface_dist:.3f}m), dist_to_panel={dist_to_panel:.3f}, dist_to_hinge={dist_to_hinge:.3f}")
            if dist_to_panel > PANEL_THRESH:
                print("PUSHING MOTION")

                # --- TARGET: fully open (same limit you used) ---
                OPEN_TARGET = -0.524
                error = OPEN_TARGET - current_door

                # ---------------------------
                # 1) STRONG DOOR TORQUE (PUSH)
                # ---------------------------
                TORQUE_GAIN = 300.0
                DAMPING = 20.0

                # Door velocity (reuse same tracker as pull)
                prev_door = getattr(run_simulator, "_prev_door_angle", current_door)
                door_vel = current_door - prev_door
                run_simulator._prev_door_angle = current_door

                # PD torque toward OPEN
                torque = TORQUE_GAIN * error - DAMPING * door_vel
                torque = float(max(min(torque, 300.0), -300.0))

                effort = torch.zeros(1, len(door_joint_names), device=palm_pos.device)
                effort[0, door_idx] = torque

                door_obj.set_joint_effort_target(effort)

                # ---------------------------
                # 2) REACTION FORCE ON HAND (PUSH BACK)
                # ---------------------------
                PALM_FORCE_GAIN = 100.0

                # Opposite of pull → push AWAY from hinge
                dir_vec = palm_pos - hinge_pos
                dir_vec = dir_vec / (torch.norm(dir_vec) + 1e-6)

                force = PALM_FORCE_GAIN * abs(error) * dir_vec

                # Apply to articulation (Isaac Lab format)
                num_bodies = robot.data.body_pos_w.shape[1]

                forces = torch.zeros((1, num_bodies, 3), device=palm_pos.device)
                torques = torch.zeros((1, num_bodies, 3), device=palm_pos.device)

                forces[0, palm_body_idx] = force

                robot.set_external_force_and_torque(
                    forces=forces,
                    torques=torques,
                )

                print(f"[PUSH FORCE] error={error:.3f} | torque={torque:.2f}")
            # -------------------------------
            # PULL: near hinge → pull back
            # -------------------------------


            else:
                print("PULLING MOTION")

                # --- TARGET: fully closed ---
                CLOSE_TARGET = 0.0
                error = CLOSE_TARGET - current_door

                # ---------------------------
                # 1) STRONG DOOR TORQUE
                # ---------------------------
                TORQUE_GAIN = 300.0   # crank this if needed
                DAMPING = 20.0        # stabilizes oscillation

                # Estimate door velocity (finite difference)
                prev_door = getattr(run_simulator, "_prev_door_angle", current_door)
                door_vel = current_door - prev_door
                run_simulator._prev_door_angle = current_door

                # PD controller → torque
                torque = TORQUE_GAIN * error - DAMPING * door_vel

                # Clamp torque
                torque = float(max(min(torque, 300.0), -300.0))

                effort = torch.zeros(1, len(door_joint_names), device=palm_pos.device)
                effort[0, door_idx] = torque

                # Apply REAL torque (not position target)
                door_obj.set_joint_effort_target(effort)

                # ---------------------------
                # 2) REACTION FORCE ON HAND
                # ---------------------------
                PALM_FORCE_GAIN = 250.0

                # Direction: hinge ← palm
                dir_vec = hinge_pos - palm_pos
                dir_vec = dir_vec / (torch.norm(dir_vec) + 1e-6)

                # Scale with how "hard" we're closing
                force = PALM_FORCE_GAIN * abs(error) * dir_vec

                # Build force tensor for ALL bodies (required API format)
                num_bodies = robot.data.body_pos_w.shape[1]

                forces = torch.zeros((1, num_bodies, 3), device=palm_pos.device)
                torques = torch.zeros((1, num_bodies, 3), device=palm_pos.device)

                # Apply ONLY to palm
                forces[0, palm_body_idx] = force

                # Apply to articulation
                robot.set_external_force_and_torque(
                    forces=forces,
                    torques=torques,
                )
                print(f"[PULL FORCE] error={error:.3f} | torque={torque:.2f}")



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
