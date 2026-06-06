import argparse
from isaaclab.app import AppLauncher
import h5py
import numpy as np
from pathlib import Path
from datetime import datetime
from isaaclab.utils.math import subtract_frame_transforms, quat_mul, quat_from_euler_xyz

"""Robot Arm Teleoperation (headless-compatible) with Task Space IK Control"""

parser = argparse.ArgumentParser(description="Robot Arm Teleoperation with Task Space IK Control")
parser.add_argument("--robot", type=str, default="franka_panda", help="Name of the robot.")
# AppLauncher adds --enable_cameras, so we don't need to add it manually here
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Force enable cameras if not set, as they are required for sensors
if not args_cli.enable_cameras:
    args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.markers import VisualizationMarkers
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import Camera, CameraCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.utils.math import subtract_frame_transforms

# Monkey-patch warp.types.array to fix Isaac Sim compatibility issue
# The installed version of warp doesn't accept 'owner' arg, but Isaac Sim passes it
try:
    import warp as wp
    if hasattr(wp.types, 'array'):
        _original_warp_array = wp.types.array
        def _patched_warp_array(*args, **kwargs):
            if 'owner' in kwargs:
                # Remove the incompatible argument
                del kwargs['owner']
            return _original_warp_array(*args, **kwargs)
        wp.types.array = _patched_warp_array
        print("[INFO] Applied Warp array monkey-patch for compatibility")
except ImportError:
    pass

from isaaclab_assets import UR10_CFG, FRANKA_PANDA_HIGH_PD_CFG

# Video recording imports
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("[WARN] OpenCV not available, video recording will use imageio instead")

try:
    import imageio
    IMAGEIO_AVAILABLE = True
except ImportError:
    IMAGEIO_AVAILABLE = False

# Import Se3Keyboard and Omni UI only if GUI mode
if not args_cli.headless:
    from isaaclab.devices import Se3Keyboard
    import omni.ui as ui


# Add recording functionality
import h5py
import numpy as np
from pathlib import Path

class DemonstrationRecorder:
    """
    Records robot demonstrations with timestamped filenames.
    Each session creates a new file: robot_demos_YYYYMMDD_HHMMSS.hdf5
    Also saves video recordings to the 'recordings' folder.
    """
    
    def __init__(self, save_dir="demonstrations", recordings_dir="recordings"):
        # Create save directories if they don't exist
        self.save_dir = Path(save_dir)
        self.save_dir.mkdir(exist_ok=True, parents=True)
        
        self.recordings_dir = Path(recordings_dir)
        self.recordings_dir.mkdir(exist_ok=True, parents=True)
        
        self.episodes = []
        
        # Determine starting episode number by scanning existing recordings
        self.episode_counter = 0
        if self.recordings_dir.exists():
            import re
            pattern = re.compile(r"episode_(\d+)_")
            max_ep = -1
            for file_path in self.recordings_dir.glob("*.mp4"):
                match = pattern.search(file_path.name)
                if match:
                    try:
                        ep_num = int(match.group(1))
                        if ep_num > max_ep:
                            max_ep = ep_num
                    except ValueError:
                        pass
            if max_ep >= 0:
                self.episode_counter = max_ep + 1
                print(f"[INFO] Found existing recordings up to episode_{max_ep}. Resuming recording from episode_{self.episode_counter}")
        
        self.current_episode = {
            'episode_num': 0,
            'observations': [],
            'actions': [],
            'ee_poses': [],
            'joint_positions': [],
            'video_frames': {}  # Dictionary of lists {cam_name: [frames]}
        }
        self.recording = False
        self.video_fps = 10  # Frames per second for saved videos
        
        print(f"[INFO] HDF5 and MP4 files will save per-episode to: {self.save_dir} and {self.recordings_dir}")
    
    def start_episode(self):
        """Start recording a new episode"""
        self.recording = True
        self.current_episode = {
            'episode_num': self.episode_counter,
            'observations': [],
            'actions': [],
            'ee_poses': [],
            'joint_positions': [],
            'video_frames': {}  # Dictionary of lists
        }
        print(f"[RECORDING] Started episode (will be episode_{self.episode_counter})")
    
    def add_transition(self, obs, action, ee_pose, joint_pos, video_frames_dict=None):
        """Add a single transition to the current episode
        
        Args:
            video_frames_dict: Dictionary mapping camera names to frame data
        """
        if self.recording:
            self.current_episode['observations'].append(obs.cpu().numpy())
            self.current_episode['actions'].append(action.cpu().numpy())
            self.current_episode['ee_poses'].append(ee_pose.cpu().numpy())
            self.current_episode['joint_positions'].append(joint_pos.cpu().numpy())
            
            # Store video frames if provided
            if video_frames_dict is not None and isinstance(video_frames_dict, dict):
                for cam_name, frame_data in video_frames_dict.items():
                    if cam_name not in self.current_episode['video_frames']:
                        self.current_episode['video_frames'][cam_name] = []
                    
                    # Handle frame data conversion if needed (though simulation loop usually handles this now)
                    if hasattr(frame_data, 'cpu'):
                        frame = frame_data.cpu().numpy()
                    elif hasattr(frame_data, 'numpy'):
                        frame = frame_data.numpy()
                    elif isinstance(frame_data, list):
                        frame = np.array(frame_data)
                    else:
                        frame = frame_data
                        
                    self.current_episode['video_frames'][cam_name].append(frame)
    
    def _save_video(self, frames, episode_num, cam_suffix=""):
        """Save video frames to MP4 file"""
        if len(frames) == 0:
            return None
        
        # Add camera suffix if provided
        name_part = f"episode_{episode_num}"
        if cam_suffix:
            name_part += f"_{cam_suffix}"
            
        video_filename = f"{name_part}.mp4"
        video_path = self.recordings_dir / video_filename
        
        try:
            if CV2_AVAILABLE:
                # Use OpenCV for video writing
                height, width = frames[0].shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                out = cv2.VideoWriter(str(video_path), fourcc, self.video_fps, (width, height))
                for frame in frames:
                    # Convert RGB to BGR for OpenCV
                    if len(frame.shape) == 3 and frame.shape[2] == 3:
                        frame_bgr = cv2.cvtColor(frame.astype(np.uint8), cv2.COLOR_RGB2BGR)
                    else:
                        frame_bgr = frame.astype(np.uint8)
                    out.write(frame_bgr)
                out.release()
                print(f"[VIDEO] Saved {len(frames)} frames to {video_path}")
            elif IMAGEIO_AVAILABLE:
                # Use imageio for video writing
                imageio.mimwrite(str(video_path), [f.astype(np.uint8) for f in frames], fps=self.video_fps)
                print(f"[VIDEO] Saved {len(frames)} frames to {video_path}")
            else:
                print("[WARN] No video library available (cv2 or imageio). Video not saved.")
                return None
            return str(video_path)
        except Exception as e:
            print(f"[ERROR] Failed to save video: {e}")
            return None
    
    def end_episode(self):
        """End the current episode, save video, and store episode data"""
        if self.recording and len(self.current_episode['observations']) > 0:
            episode_num = self.current_episode['episode_num']
            num_steps = len(self.current_episode['observations'])
            
            # Save videos for all cameras
            video_paths = {}
            total_frames = 0
            
            for cam_name, frames in self.current_episode['video_frames'].items():
                if len(frames) > 0:
                    path = self._save_video(frames, episode_num, cam_suffix=cam_name)
                    if path:
                        video_paths[cam_name] = path
                        total_frames += len(frames)
            
            # Store video path reference and clear frames from memory
            self.current_episode['video_paths'] = video_paths
            self.current_episode['video_frames'] = {}  # Clear frames to save memory
            
            self.episodes.append(self.current_episode)
            self.episode_counter += 1  # Increment global counter
            print(f"[RECORDING] Episode {episode_num} completed with {num_steps} steps. Saved {len(video_paths)} videos.")
            
            # Automatically save the HDF5 file alongside the MP4s
            self.save()
            self.get_quick_summary()
        elif self.recording:
            print("[WARN] Episode ended but no data was recorded")
        self.recording = False
    
    def save(self):
        """Save each episode from this session to its own HDF5 file"""
        if len(self.episodes) == 0:
            print("[WARN] No episodes to save")
            return
        
        saved_count = 0
        try:
            for episode in self.episodes:
                episode_num = episode.get('episode_num', 0)
                file_path = self.save_dir / f"robot_demos_{episode_num}.hdf5"
                
                # Use 'w' mode as each episode gets its own new file
                with h5py.File(file_path, 'w') as f:
                    grp = f.create_group(f'episode_{episode_num}')
                    for key, value in episode.items():
                        # Skip video_frames (already saved as MP4) and internal metadata
                        if key in ('video_frames', 'episode_num'):
                            continue
                        elif key == 'video_path':
                             grp.attrs['video_path'] = value if value else ""
                        elif key == 'video_paths':
                            # Store paths for all cameras
                            for cam_name, path in value.items():
                                grp.attrs[f'video_path_{cam_name}'] = str(path)
                        else:
                            grp.create_dataset(key, data=np.array(value))
                saved_count += 1
            
            print(f"[SUCCESS] Saved {saved_count} episodes individual files to {self.save_dir.absolute()}")
            
            # Clear episodes from memory after successful save
            self.episodes = []
            
        except Exception as e:
            print(f"[ERROR] Failed to save demonstrations: {e}")
    
    def get_stats(self, detailed=False):
        """Print comprehensive statistics about recorded demonstrations."""
        print("\n" + "="*60)
        print("DEMONSTRATION STATISTICS")
        print("="*60)
        
        print(f"\n📁 Saving format: 1 file per episode (e.g., robot_demos_0.hdf5)")
        
        # Episodes in memory
        episodes_in_memory = [len(ep['observations']) for ep in self.episodes]
        total_in_memory = len(self.episodes)
        
        print(f"\n📊 This Session:")
        print(f"   Episodes recorded: {total_in_memory}")
        
        if episodes_in_memory:
            total_timesteps = sum(episodes_in_memory)
            avg_length = np.mean(episodes_in_memory)
            min_length = min(episodes_in_memory)
            max_length = max(episodes_in_memory)
            
            print(f"\n📈 Episode Statistics:")
            print(f"   Total timesteps: {total_timesteps:,}")
            print(f"   Average: {avg_length:.1f} steps")
            print(f"   Min: {min_length} steps")
            print(f"   Max: {max_length} steps")
            
            if detailed:
                print(f"\n📋 Per-Episode Breakdown:")
                for i, steps in enumerate(episodes_in_memory):
                    print(f"   episode_{i} in memory: {steps} steps (unsaved)")
        
        # Show all files in directory
        if self.save_dir.exists():
            all_files = sorted(self.save_dir.glob("robot_demos_*.hdf5"))
            if all_files:
                print(f"\n📂 All Sessions in {self.save_dir}:")
                total_all_episodes = 0
                for file_path in all_files:
                    try:
                        with h5py.File(file_path, 'r') as f:
                            num_eps = len(f.keys())
                            total_all_episodes += num_eps
                            print(f"   {file_path.name}: {num_eps} episodes")
                    except:
                        print(f"   {file_path.name}: Error reading")
                print(f"\n   Total episodes across all sessions: {total_all_episodes}")
        
        print(f"\n🎯 Training Readiness:")
        if total_in_memory > 0:
            print(f"   ⚠ WARNING: {total_in_memory} unsaved episodes!")
            print(f"   → Click 'Save All Demos' to save them")
        else:
            print(f"   ✓ All episodes saved")
        
        print("="*60 + "\n")
    
    def get_quick_summary(self):
        """Print a quick one-line summary"""
        unsaved = len(self.episodes)
        status = "✓" if unsaved == 0 else f"⚠ {unsaved} unsaved"
        print(f"[STATS] This session: {len(self.episodes)} episodes {status}")

def _get_cam_quat(angle_deg):
    try:
        angle_rad = np.deg2rad(angle_deg)
        q = quat_from_euler_xyz(
            torch.tensor([0.0]),
            torch.tensor([np.deg2rad(30.0)]),
            torch.tensor([angle_rad + np.pi])
        )
        return tuple(q[0].tolist())
    except:
        return (1.0, 0.0, 0.0, 0.0)

@configclass
class TableTopSceneCfg(InteractiveSceneCfg):
    """Configuration for a simple tabletop scene with a robot."""

    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/Stand/stand_instanceable.usd", scale=(2.0, 2.0, 2.0)
        ),
    )

    # Elevated platform/table for the pickable cube
    cube_table = AssetBaseCfg(
        prim_path="/World/CubeTable",
        spawn=sim_utils.CuboidCfg(
            size=[0.4, 0.4, 0.6],  # 40cm x 40cm surface, 60cm tall
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                kinematic_enabled=True,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.8, 0.6, 0.4),  # Wood-like color
            ),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.8, 0, -0.1)),  # Moved away from origin, base at ground
    )

    # Original cube
    cube = AssetBaseCfg(
        prim_path="/World/cube",
        spawn=sim_utils.CuboidCfg(size=[0.1, 0.1, 0.1]),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.5, 0.0, 0.5)),
    )

    # Second pickable cube - now on elevated table
    cube2 = AssetBaseCfg(
        prim_path="/World/cube2",
        spawn=sim_utils.CuboidCfg(
            size=[0.03, 0.03, 0.03],
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=1.0,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.2),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.2, 0.6, 0.9),  # Blue color for visibility
            ),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.8, 0, 0.23)),  # On top of table, matching table position
    )

    if args_cli.robot == "franka_panda":
        robot = FRANKA_PANDA_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    elif args_cli.robot == "ur10":
        robot = UR10_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    else:
        raise ValueError(f"Robot {args_cli.robot} is not supported. Valid: franka_panda, ur10")
    
    # Main Camera (Camera 1 position - user's preferred angle)
    # Position: 36 degrees around circle, radius 1.5m from (0.5, 0), height 1.0m
    camera = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Camera",
        update_period=0.1,  # 10 Hz
        height=480,
        width=640,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            focus_distance=400.0,
            horizontal_aperture=20.955,
            clipping_range=(0.1, 20.0),
        ),
        offset=CameraCfg.OffsetCfg(
            pos=(1.71, 0.88, 1.0),  # Camera 1 position (36 deg)
            rot=_get_cam_quat(36.0),
            convention="world",
        ),
    )

    # Camera 3 (108 degrees - left side view)
    camera_3 = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Camera_3",
        update_period=0.1,
        height=480,
        width=640,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            focus_distance=400.0,
            horizontal_aperture=20.955,
            clipping_range=(0.1, 20.0),
        ),
        offset=CameraCfg.OffsetCfg(
            pos=(0.04, 1.43, 1.0),  # Camera 3 position (108 deg)
            rot=_get_cam_quat(108.0),
            convention="world",
        ),
    )

    # Camera 9 (324 degrees - right side view, opposite of Camera 3)
    camera_9 = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Camera_9",
        update_period=0.1,
        height=480,
        width=640,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            focus_distance=400.0,
            horizontal_aperture=20.955,
            clipping_range=(0.1, 20.0),
        ),
        offset=CameraCfg.OffsetCfg(
            pos=(1.71, -0.88, 1.0),  # Camera 9 position (324 deg)
            rot=_get_cam_quat(324.0),
            convention="world",
        ),
    )

    # Visual marker for main camera (so you can see where it is)
    camera_marker = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/CameraMarker",
        spawn=sim_utils.CuboidCfg(
            size=[0.1, 0.1, 0.2],  # Small box representing camera
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),  # Red color
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(1.71, 0.88, 1.0),  # MATCHES MAIN CAMERA POSITION
            rot=_get_cam_quat(36.0),
        ),
    )



# Note: Camera 1 (main), Camera 3, and Camera 9 are now defined inside the class above.
# The dynamic loop has been removed for cleaner configuration.


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    robot = scene["robot"]
    
    # Enable collisions for gripper to allow grasping
    print("[INFO] Enabling gripper collisions for object interaction...")

    diff_ik_cfg = DifferentialIKControllerCfg(command_type="position", use_relative_mode=True, ik_method="dls")
    diff_ik_controller = DifferentialIKController(diff_ik_cfg, num_envs=scene.num_envs, device=sim.device)

    # -----------------------
    # SMOOTH APPROACH PARAMETERS
    # -----------------------
    position_smoothing = 0.15
    rotation_smoothing = 0.12
    max_linear_velocity = 0.6
    max_angular_velocity = 1.0
    slow_zone_threshold = 0.15
    min_speed_ratio = 0.1
    position_deadband = 0.002
    rotation_deadband = 0.02

    frame_marker_cfg = FRAME_MARKER_CFG.copy()
    frame_marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
    ee_marker = VisualizationMarkers(frame_marker_cfg.replace(prim_path="/Visuals/ee_current"))
    goal_marker = VisualizationMarkers(frame_marker_cfg.replace(prim_path="/Visuals/ee_goal"))

    if args_cli.robot == "franka_panda":
        robot_entity_cfg = SceneEntityCfg("robot", joint_names=["panda_joint.*"], body_names=["panda_hand"])
    elif args_cli.robot == "ur10":
        robot_entity_cfg = SceneEntityCfg("robot", joint_names=[".*"], body_names=["ee_link"])
    robot_entity_cfg.resolve(scene)

    ee_jacobi_idx = robot_entity_cfg.body_ids[0] - 1 if robot.is_fixed_base else robot_entity_cfg.body_ids[0]
    sim_dt = 0.01

    # Initialize joint states
    if args_cli.robot == "franka_panda":
        joint_position = robot.data.default_joint_pos.clone()
        joint_vel = robot.data.default_joint_vel.clone()
        robot.write_joint_state_to_sim(joint_position, joint_vel)
    else:
        joint_position = torch.zeros((1, 6), device=sim.device)
        joint_vel = robot.data.default_joint_vel.clone()
        robot.write_joint_state_to_sim(joint_position, joint_vel)

    # -----------------------
    # GRIPPER SETUP
    # -----------------------
    joint_names = [n.lower() for n in robot.data.joint_names]
    gripper_candidates = [i for i, n in enumerate(joint_names) if any(k in n for k in ["finger", "gripper", "hand", "claw", "panda_finger"])]
    gripper_joint_ids = gripper_candidates

    if len(gripper_joint_ids) > 0:
        print("[INFO] Detected gripper joints:", [robot.data.joint_names[i] for i in gripper_joint_ids])
    else:
        print("[WARN] No gripper joints detected. Gripper control will be disabled.")

    gripper_open_pos = 0.04
    gripper_closed_pos = 0.0
    gripper_target_norm = 0.0
    gripper_open_bool = True

    def gripper_norm_to_joint_positions(norm):
        pos = gripper_open_pos + (gripper_closed_pos - gripper_open_pos) * norm
        if len(gripper_joint_ids) == 0:
            return None
        return torch.tensor([[pos] * len(gripper_joint_ids)], device=sim.device)

    # ========================================================================
    # ADD DEMONSTRATION RECORDER HERE - STEP 1: Initialize recorder
    # ========================================================================
    recorder = DemonstrationRecorder("demonstrations", "recordings")
    print("[INFO] Demonstration recorder initialized")
    
    # Get camera from scene for video recording
    print(f"[DEBUG] Scene keys: {list(scene.keys())}")
    camera = scene["camera"] if "camera" in scene.keys() else None
    if camera is not None:
        print(f"[INFO] Camera sensor found in scene")
        # Don't access camera.data here as it might not be initialized yet
    else:
        print("[WARN] No camera sensor found in scene. Video recording disabled.")

    # -----------------------
    # TELEOP / HEADLESS SETUP
    # -----------------------
    if not args_cli.headless:
        try:
            teleop = Se3Keyboard(pos_sensitivity=0.15, rot_sensitivity=0.05)
        except TypeError:
            try:
                teleop = Se3Keyboard(0.05, 0.05)
            except TypeError:
                teleop = Se3Keyboard()
                teleop.pos_sensitivity = 0.05
                teleop.rot_sensitivity = 0.05
        teleop.reset()
        print("[INFO] Teleoperation active — use WASDQE to move and arrow keys to rotate.")
        print("[INFO] Smooth approach enabled: arm will slow down near goal")
        teleop_has_extra_keys = False

        gripper_state = {"open": True}

        def _toggle_gripper_cb():
            gripper_state["open"] = not gripper_state["open"]
            print(f"[UI] Gripper toggled -> {'OPEN' if gripper_state['open'] else 'CLOSED'}")

        # ========================================================================
        # ADD RECORDING CONTROLS HERE - STEP 2: Add UI buttons for recording
        # ========================================================================
        def _start_recording():
            recorder.start_episode()
            print("[RECORDING] Started new episode - perform your demonstration")

        def _stop_recording():
            recorder.end_episode()
            print("[RECORDING] Stopped episode and saved HDF5 data")

        # Create UI windows
        gripper_window = ui.Window("Gripper", width=180, height=80)
        with gripper_window.frame:
            with ui.VStack(spacing=10):
                ui.Label("Gripper Control")
                ui.Button("Toggle Gripper", clicked_fn=_toggle_gripper_cb, height=40)

        # Recording control window
        recording_window = ui.Window("Recording", width=180, height=110, position_x=200)
        with recording_window.frame:
            with ui.VStack(spacing=10):
                ui.Label("Demonstration Recording")
                ui.Button("Start Recording", clicked_fn=_start_recording, height=40)
                ui.Button("Stop Recording / Save", clicked_fn=_stop_recording, height=40)

    else:
        step = 0
        headless_cycle_t = 0
        print("[INFO] Running headless simulation with scripted motion + gripper sequence...")

    #ee_pose_w = robot.data.body_state_w[:, robot_entity_cfg.body_ids[0], 0:7]
    #goal_pose = ee_pose_w.clone()
    #smooth_target_pose = goal_pose.clone()
    #previous_smooth_pose = smooth_target_pose.clone()

    ee_command = torch.zeros(1, 3, device=sim.device)

    # Main simulation loop
    while simulation_app.is_running():
        if not args_cli.headless:
            ret = teleop.advance()
            if isinstance(ret, tuple) and len(ret) == 3:
                pos_delta, rot_delta, _ = ret
            else:
                pos_delta, rot_delta = ret

            # Zero the command each step — only non-zero when a key is held
            ee_command = torch.zeros(1, 3, device=sim.device)

            if isinstance(pos_delta, (list, tuple, np.ndarray)):
                ee_command[0, 0:3] = torch.tensor(pos_delta[:3], dtype=torch.float32, device=sim.device)
            gripper_open_bool = gripper_state["open"]
            gripper_target_norm = 0.0 if gripper_open_bool else 1.0

        else:
            # Headless scripted motion — send a small sinusoidal x delta
            ee_command = torch.zeros(1, 3, device=sim.device)
            ee_command[0, 0] = 0.002 * float(torch.sin(torch.tensor(step * 0.1)))
            step += 1

            headless_cycle_t += 1
            cycle_len = 200
            tmod = headless_cycle_t % cycle_len
            gripper_target_norm = 0.0 if tmod < (cycle_len // 2) else 1.0
            gripper_open_bool = gripper_target_norm == 0.0

        # -----------------------
        # SMOOTH APPROACH LOGIC
        # -----------------------
        
        '''position_error = goal_pose[:, 0:3] - smooth_target_pose[:, 0:3]
        distance_to_goal = torch.norm(position_error, dim=1, keepdim=True)
        
        speed_scale = torch.clamp(
            distance_to_goal / slow_zone_threshold,
            min=min_speed_ratio,
            max=1.0
        )
        
        if distance_to_goal.item() > position_deadband:
            position_delta = position_smoothing * speed_scale * position_error
            max_position_delta = max_linear_velocity * sim_dt
            position_delta_norm = torch.norm(position_delta)
            if position_delta_norm > max_position_delta:
                position_delta = position_delta / position_delta_norm * max_position_delta
            smooth_target_pose[:, 0:3] += position_delta
        
        goal_quat = goal_pose[:, 3:7]
        current_quat = smooth_target_pose[:, 3:7]
        dot_product = torch.sum(goal_quat * current_quat, dim=1, keepdim=True)
        goal_quat_corrected = torch.where(dot_product < 0, -goal_quat, goal_quat)
        rotation_error = goal_quat_corrected - current_quat
        rotation_error_magnitude = torch.norm(rotation_error, dim=1, keepdim=True)
        
        if rotation_error_magnitude.item() > rotation_deadband:
            rotation_delta = rotation_smoothing * rotation_error
            max_rotation_delta = max_angular_velocity * sim_dt
            rotation_delta_norm = torch.norm(rotation_delta)
            if rotation_delta_norm > max_rotation_delta:
                rotation_delta = rotation_delta / rotation_delta_norm * max_rotation_delta
            smooth_target_pose[:, 3:7] += rotation_delta
            smooth_target_pose[:, 3:7] = smooth_target_pose[:, 3:7] / torch.norm(smooth_target_pose[:, 3:7], dim=1, keepdim=True)
        
        alpha = 0.7
        smooth_target_pose = alpha * smooth_target_pose + (1 - alpha) * previous_smooth_pose
        smooth_target_pose[:, 3:7] = smooth_target_pose[:, 3:7] / torch.norm(smooth_target_pose[:, 3:7], dim=1, keepdim=True)
        previous_smooth_pose = smooth_target_pose.clone()'''

        # IK computation
        # diff_ik_controller.set_command(smooth_target_pose)
        # IK computation — compute EE pose FIRST, then set command
        ee_pose_w = robot.data.body_state_w[:, robot_entity_cfg.body_ids[0], 0:7]
        root_pose_w = robot.data.root_state_w[:, 0:7]
        joint_pos = robot.data.joint_pos[:, robot_entity_cfg.joint_ids]

        ee_pos_b, ee_quat_b = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7],
            ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
        )

        # Pass current orientation so controller can hold it fixed
        diff_ik_controller.set_command(ee_command, ee_pos=ee_pos_b, ee_quat=ee_quat_b)

        jacobian = robot.root_physx_view.get_jacobians()[:, ee_jacobi_idx, :, robot_entity_cfg.joint_ids]
        joint_pos_des = diff_ik_controller.compute(ee_pos_b, ee_quat_b, jacobian, joint_pos)
        # Set arm joint targets
        robot.set_joint_position_target(joint_pos_des, joint_ids=robot_entity_cfg.joint_ids)

        # Apply gripper joint targets
        if len(gripper_joint_ids) > 0:
            gripper_joint_positions = gripper_norm_to_joint_positions(gripper_target_norm)
            if gripper_joint_positions is not None:
                robot.set_joint_position_target(
                    gripper_joint_positions,
                    joint_ids=gripper_joint_ids
                )

        # ========================================================================
        # RECORD DEMONSTRATION DATA HERE - STEP 3: Record transitions
        # ========================================================================
        # Create observation (matching what you'll use for training)
        joint_pos_full = robot.data.joint_pos[:, robot_entity_cfg.joint_ids]
        joint_vel_full = robot.data.joint_vel[:, robot_entity_cfg.joint_ids]
        
        # Observation: [joint_pos(7), joint_vel(7), ee_pose(7), gripper_state(1)]
        obs = torch.cat([
            joint_pos_full,
            joint_vel_full,
            ee_pose_w,
            torch.tensor([[gripper_target_norm]], device=sim.device)
        ], dim=-1)
        
        # Action: joint position targets + gripper target
        action = torch.cat([
            ee_command,  # 3D: [dx, dy, dz]
            torch.tensor([[gripper_target_norm]], device=sim.device)
        ], dim=-1)
        
        # Record the transition
        # Record the transition
        # Capture frames from all cameras
        video_frames_dict = {}
        
        # Helper to capture from a single camera sensor
        def _get_camera_frame(sensor):
            if sensor is None: return None
            try:
                rgb_data = sensor.data.output["rgb"]
                if rgb_data is None: return None
                
                # Handle different tensor formats
                if hasattr(rgb_data, 'shape') and len(rgb_data.shape) >= 3:
                     # Get first environment if batched
                     frame_data = rgb_data[0] if len(rgb_data.shape) == 4 else rgb_data
                     
                     # Convert to numpy
                     if hasattr(frame_data, 'cpu'):
                         v_frame = frame_data.clone().cpu().numpy()
                     elif hasattr(frame_data, 'numpy'):
                         v_frame = frame_data.numpy().copy()
                     else:
                         v_frame = np.asarray(frame_data).copy()
                         
                     # Ensure uint8
                     if v_frame.dtype != np.uint8:
                         v_frame = (v_frame * 255).astype(np.uint8) if v_frame.max() <= 1.0 else v_frame.astype(np.uint8)
                     return v_frame
            except Exception as e:
                # Use a flag to print error only once per sensor
                if not hasattr(sensor, '_error_printed'):
                    print(f"[WARN] Frame capture error for {sensor}: {e}")
                    sensor._error_printed = True
            return None

        # Iterate over all likely camera sensors in the scene
        for key in scene.keys():
            # Skip markers, look for 'camera' in name (case insensitive)
            if "camera" in key.lower() and "marker" not in key.lower():
                try:
                    sensor = scene[key]
                    # Duck typing check for data.output
                    if hasattr(sensor, 'data') and hasattr(sensor.data, 'output'):
                         frame = _get_camera_frame(sensor)
                         if frame is not None:
                             video_frames_dict[key] = frame
                except:
                    pass
        
        recorder.add_transition(
            obs=obs,
            action=action,
            ee_pose=ee_pose_w,
            joint_pos=joint_pos_full,
            video_frames_dict=video_frames_dict
        )

        # Write/update simulation
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

        # Visualize EE & goal
        ee_pose_w = robot.data.body_state_w[:, robot_entity_cfg.body_ids[0], 0:7]
        ee_marker.visualize(ee_pose_w[:, 0:3], ee_pose_w[:, 3:7])
        #goal_marker.visualize(goal_pose[:, 0:3] + scene.env_origins, goal_pose[:, 3:7])

def main():
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)

    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])
    scene_cfg = TableTopSceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    print("[INFO]: Setup complete...")

    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()