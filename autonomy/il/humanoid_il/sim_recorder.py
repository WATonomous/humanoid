"""GPU-buffered async LeRobot dataset recorder for Isaac Sim teleop."""
from __future__ import annotations

import queue
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
import torch
from tqdm import tqdm

from humanoid_il.episode_keys import EpisodeFlags, EpisodeKeyboard


class SimLeRobotRecorder:
    """Buffer frames in GPU tensors, flush to a LeRobot dataset asynchronously.

    Designed for Isaac Sim where observations are already on-device — batching
    the PCIe transfer to one copy-per-episode avoids per-frame overhead.

    Usage::

        recorder = SimLeRobotRecorder(...)
        recorder.init_dataset()
        # inside sim loop:
        recorder.push_frame_to_buffer(action, obs, visual_buffers)
        # on episode end:
        recorder.save_episode()   # enqueues async save, clears buffers
        # on cancel:
        recorder.cancel_recording()
        # after all episodes:
        recorder.finalize()
    """

    def __init__(
        self,
        task_name: str,
        repo_id: str,
        dataset_root: str | Path,
        fps: int,
        device: str,
        joint_names: list[str],
        cameras: dict[str, dict[str, int]],
        save_mp4: bool = False,
        depth: bool = False,
        instance_id_seg: bool = False,
        num_episodes: int | None = None,
        buffer_capacity_s: float = 120.0,
    ) -> None:
        self.fps = fps
        self.save_mp4 = save_mp4
        self.depth = depth
        self.instance_id_seg = instance_id_seg
        self.device = device
        self.joint_names = list(joint_names)
        self.cameras = cameras
        self.repo_id = repo_id
        self.dataset_root = Path(dataset_root)
        self.task_name = task_name
        self.num_episodes = num_episodes
        self.num_recorded_episodes = 0

        self._capacity = int(buffer_capacity_s * fps)
        self._current_frame = 0

        self._action_buf: torch.Tensor | None = None
        self._obs_buf: torch.Tensor | None = None
        self._rgb_bufs: dict[str, torch.Tensor] = {}
        self._depth_bufs: dict[str, torch.Tensor] = {}
        self._seg_bufs: dict[str, torch.Tensor] = {}

        self._episode_queue: queue.Queue = queue.Queue()
        self._stop_event = threading.Event()
        self._processor_thread = threading.Thread(
            target=self._async_processor, daemon=True
        )
        self._processor_thread.start()

        self._flags: EpisodeFlags | None = None
        self._keyboard: EpisodeKeyboard | None = None
        self._last_frame_t: float = 0.0
        self._frame_period: float = 1.0 / fps

    def start_keyboard(self) -> bool:
        """Create episode flags, attach keyboard listener, and return whether keyboard started."""
        self._flags = EpisodeFlags(start=False)
        self._keyboard = EpisodeKeyboard(self._flags)
        return self._keyboard.start()

    def tick(
        self,
        action: np.ndarray | torch.Tensor,
        state: np.ndarray | torch.Tensor,
        images: dict[str, np.ndarray | torch.Tensor],
        depth_buffers: dict[str, np.ndarray | torch.Tensor] | None = None,
        instance_id_seg_buffers: dict[str, np.ndarray | torch.Tensor] | None = None,
    ) -> bool:
        """Handle episode flags and push one frame if the rate allows.

        Returns True if an episode was just saved (so callers can trigger a scene reset).
        No-op if start_keyboard() was never called.
        """
        if self._flags is None:
            return False
        flags = self._flags
        if flags.remove:
            self.cancel_recording()
            flags.remove = False
        if flags.success:
            self.save_episode()
            flags.success = False
            flags.start = False
            return True
        if flags.start:
            now = time.monotonic()
            if now - self._last_frame_t >= self._frame_period:
                self._last_frame_t = now
                self.push_frame_to_buffer(action, state, images, depth_buffers, instance_id_seg_buffers)
        return False

    @property
    def is_complete(self) -> bool:
        return (
            self.num_episodes is not None
            and self.num_recorded_episodes >= self.num_episodes
        )

    def _build_features(self) -> dict[str, Any]:
        dim = len(self.joint_names)
        features: dict[str, Any] = {
            "observation.state": {
                "dtype": "float32",
                "fps": self.fps,
                "shape": (dim,),
                "names": self.joint_names,
            },
            "action": {
                "dtype": "float32",
                "fps": self.fps,
                "shape": (dim,),
                "names": self.joint_names,
            },
        }
        for name, spec in self.cameras.items():
            features[f"observation.images.{name}"] = {
                "dtype": "video",
                "shape": (spec["height"], spec["width"], 3),
                "names": ["height", "width", "channels"],
            }
        return features

    def init_dataset(self) -> None:
        """Create or re-open the LeRobot dataset on disk."""
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        root = self.dataset_root
        if root.exists():
            try:
                self.dataset = LeRobotDataset(self.repo_id, root=root)
                print(f"[INFO]: Opened existing dataset at {root}")
                return
            except Exception:
                raise ValueError(
                    f"[ERROR]: Dataset folder exists but cannot be opened: {root}"
                )

        self.dataset = LeRobotDataset.create(
            self.repo_id,
            fps=self.fps,
            features=self._build_features(),
            root=root,
            robot_type="so101_follower",
        )
        print(f"[INFO]: Created new dataset at {root}")

    def _allocate_buffers(self) -> None:
        dim = len(self.joint_names)
        cap, dev = self._capacity, self.device
        self._action_buf = torch.zeros((cap, dim), dtype=torch.float32, device=dev)
        self._obs_buf = torch.zeros((cap, dim), dtype=torch.float32, device=dev)
        for name, spec in self.cameras.items():
            h, w = spec["height"], spec["width"]
            self._rgb_bufs[name] = torch.zeros(
                (cap, h, w, 3), dtype=torch.uint8, device=dev
            )
            if self.depth:
                self._depth_bufs[name] = torch.zeros(
                    (cap, h, w, 1), dtype=torch.float32, device=dev
                )
            if self.instance_id_seg:
                self._seg_bufs[name] = torch.zeros(
                    (cap, h, w, 3), dtype=torch.uint8, device=dev
                )

    @staticmethod
    def _as_tensor(
        x: np.ndarray | torch.Tensor, dtype: torch.dtype, device: str
    ) -> torch.Tensor:
        if isinstance(x, np.ndarray):
            return torch.from_numpy(np.ascontiguousarray(x)).to(
                device=device, dtype=dtype
            )
        return x.to(device=device, dtype=dtype)

    def push_frame_to_buffer(
        self,
        action: np.ndarray | torch.Tensor,
        observation: np.ndarray | torch.Tensor,
        visual_buffers: dict[str, np.ndarray | torch.Tensor],
        depth_buffers: dict[str, np.ndarray | torch.Tensor] | None = None,
        instance_id_seg_buffers: dict[str, np.ndarray | torch.Tensor] | None = None,
    ) -> None:
        """Push one timestep of data into the GPU buffers."""
        if self._current_frame >= self._capacity:
            print(
                f"[WARN]: Buffer full at frame {self._current_frame}, skipping"
            )
            return
        if self._action_buf is None:
            self._allocate_buffers()

        i = self._current_frame
        self._action_buf[i] = self._as_tensor(action, torch.float32, self.device)
        self._obs_buf[i] = self._as_tensor(observation, torch.float32, self.device)

        for name in self.cameras:
            self._rgb_bufs[name][i] = self._as_tensor(
                visual_buffers[name], torch.uint8, self.device
            )
            if self.depth and depth_buffers:
                self._depth_bufs[name][i] = self._as_tensor(
                    depth_buffers[name], torch.float32, self.device
                )
            if self.instance_id_seg and instance_id_seg_buffers:
                self._seg_bufs[name][i] = self._as_tensor(
                    instance_id_seg_buffers[name], torch.uint8, self.device
                )

        self._current_frame += 1

    def save_episode(self) -> None:
        """Batch-copy the current episode to CPU and enqueue for async saving."""
        print("[INFO]: Copying episode to CPU...")
        n = self._current_frame
        episode: dict[str, Any] = {
            "total_frames": n,
            "action": (
                self._action_buf[:n].cpu().numpy().copy()
                if self._action_buf is not None
                else np.zeros((0, len(self.joint_names)), dtype=np.float32)
            ),
            "observation": (
                self._obs_buf[:n].cpu().numpy().copy()
                if self._obs_buf is not None
                else np.zeros((0, len(self.joint_names)), dtype=np.float32)
            ),
            "rgb": {
                name: self._rgb_bufs[name][:n].cpu().numpy().copy()
                for name in self.cameras
            },
        }
        if self.depth:
            episode["depth"] = {
                name: self._depth_bufs[name][:n].cpu().numpy().copy()
                for name in self.cameras
            }
        if self.instance_id_seg:
            episode["seg"] = {
                name: self._seg_bufs[name][:n].cpu().numpy().copy()
                for name in self.cameras
            }

        self._episode_queue.put(episode)
        self._clear_buffers()
        print("[INFO]: Episode queued for saving.")

    def cancel_recording(self) -> None:
        """Discard the current episode buffer without saving."""
        self._clear_buffers()
        print("[INFO]: Recording cancelled.")

    def _clear_buffers(self) -> None:
        self._action_buf = None
        self._obs_buf = None
        self._rgb_bufs = {}
        self._depth_bufs = {}
        self._seg_bufs = {}
        self._current_frame = 0

    def _async_processor(self) -> None:
        while not self._stop_event.is_set():
            try:
                episode = self._episode_queue.get(timeout=1.0)
            except queue.Empty:
                continue
            try:
                self._process_episode(episode)
                self.num_recorded_episodes += 1
                print(f"[INFO]: Episode {self.num_recorded_episodes} saved.")
            except Exception as exc:
                print(f"[ERROR]: Episode processing failed: {exc}")
            finally:
                self._episode_queue.task_done()

    def _process_episode(self, episode: dict[str, Any]) -> None:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        n = episode["total_frames"]
        for i in tqdm(range(n), desc="Processing frames", unit="frame"):
            frame: dict[str, Any] = {
                "action": episode["action"][i],
                "observation.state": episode["observation"][i],
                "task": self.task_name,
            }
            for name in self.cameras:
                frame[f"observation.images.{name}"] = episode["rgb"][name][i]
            self.dataset.add_frame(frame)

        if self.save_mp4:
            for name in self.cameras:
                self._save_rgb_video(episode["rgb"][name][:n], name)
                if self.depth and "depth" in episode:
                    self._save_depth_video(episode["depth"][name][:n], name)
                if self.instance_id_seg and "seg" in episode:
                    self._save_seg_video(episode["seg"][name][:n], name)

        self.dataset.save_episode()
        self.dataset.finalize()
        self.dataset = LeRobotDataset(self.repo_id, root=self.dataset_root)

    def finalize(self) -> None:
        """Block until all queued episodes are saved, then stop the worker thread."""
        if self._keyboard is not None:
            self._keyboard.stop()
            self._keyboard = None
        self._episode_queue.join()
        self._stop_event.set()
        self._processor_thread.join(timeout=5.0)

    # --- video helpers ---

    def _save_video(
        self, frames_rgb: np.ndarray, camera_name: str, data_type: str
    ) -> None:
        ep = self.num_recorded_episodes + 1
        out = (
            self.dataset_root
            / "mp4"
            / self.repo_id.split("/")[-1]
            / camera_name
            / f"{camera_name}_{data_type}_{ep:03d}.mp4"
        )
        out.parent.mkdir(parents=True, exist_ok=True)
        _, h, w, _ = frames_rgb.shape
        cmd = [
            "ffmpeg", "-y",
            "-f", "rawvideo", "-vcodec", "rawvideo",
            "-s", f"{w}x{h}", "-pix_fmt", "rgb24", "-r", str(self.fps),
            "-i", "-",
            "-c:v", "libx264", "-preset", "medium", "-crf", "23",
            "-pix_fmt", "yuv420p", str(out),
        ]
        try:
            proc = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            _, stderr = proc.communicate(
                input=b"".join(f.tobytes() for f in frames_rgb)
            )
            if proc.returncode != 0:
                print(f"[ERROR]: ffmpeg failed: {stderr.decode()}")
            else:
                print(f"[INFO]: Saved {data_type} video → {out}")
        except FileNotFoundError:
            print("[ERROR]: ffmpeg not found")

    def _save_depth_video(self, frames: np.ndarray, camera_name: str) -> None:
        if frames.ndim == 4 and frames.shape[-1] == 1:
            frames = frames.squeeze(-1)
        valid = np.isfinite(frames)
        lo = np.percentile(frames[valid], 1) if valid.any() else 0.0
        hi = np.percentile(frames[valid], 99) if valid.any() else 1.0
        if hi - lo < 1e-6:
            hi = lo + 1.0
        norm = np.clip((frames - lo) / (hi - lo), 0, 1)
        rgb = np.stack([(norm * 255).astype(np.uint8)] * 3, axis=-1)
        self._save_video(rgb, camera_name, "depth")

    def _save_rgb_video(self, frames: np.ndarray, camera_name: str) -> None:
        self._save_video(frames.astype(np.uint8), camera_name, "rgb")

    def _save_seg_video(self, frames: np.ndarray, camera_name: str) -> None:
        self._save_video(frames.astype(np.uint8), camera_name, "instance_id_segmentation")

    def __del__(self) -> None:
        self._stop_event.set()
        if hasattr(self, "_processor_thread"):
            self._processor_thread.join(timeout=0.1)
