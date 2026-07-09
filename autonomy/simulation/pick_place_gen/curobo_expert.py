"""cuRobo motion-planning expert for the wato_bimanual_arm left arm.

Wraps the new cuRobo MotionPlanner (2025 API): world model = table slab +
object cuboids (matching the Isaac scene), plan_grasp for the pick phase,
plan_pose for transit/place segments. All poses are wrist (link6l) poses in
the robot base frame; use tip_to_wrist() to convert fingertip-center goals.

Note: the grasped object is NOT attached to the planner model during
transit (v1 simplification) — it is removed from the world model instead,
and clearance comes from MotionParams.lift_height/place_clearance.
"""
import math

import numpy as np
import torch

import wato_constants as wc
from task_params import PickPlaceTaskParams


def top_down_wrist_quat(yaw: float) -> np.ndarray:
    """World-from-wrist quaternion (wxyz): approach axis (wrist -Y) points
    down (world -Z), wrist X horizontal at `yaw`."""
    c, s = math.cos(yaw), math.sin(yaw)
    m = np.array([[c, 0.0, s], [s, 0.0, -c], [0.0, 1.0, 0.0]])
    w = math.sqrt(max(1.0 + m[0, 0] + m[1, 1] + m[2, 2], 1e-12)) / 2.0
    return np.array([
        w,
        (m[2, 1] - m[1, 2]) / (4 * w),
        (m[0, 2] - m[2, 0]) / (4 * w),
        (m[1, 0] - m[0, 1]) / (4 * w),
    ])


def quat_to_mat(q) -> np.ndarray:
    w, x, y, z = [float(v) for v in q]
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def tip_to_wrist(tip_pos: np.ndarray, wrist_quat: np.ndarray) -> np.ndarray:
    """Wrist position that puts the fingertip center at tip_pos."""
    return np.asarray(tip_pos) - quat_to_mat(wrist_quat) @ np.array(wc.FINGERTIP_OFFSET_IN_WRIST)


class CuRoboExpert:
    def __init__(self, params: PickPlaceTaskParams, device: str = "cuda"):
        from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
        from curobo.scene import Cuboid, Scene

        self.params = params
        self.device = device
        self._Cuboid = Cuboid
        self._Scene = Scene

        self._table = Cuboid(
            name="table",
            dims=list(wc.TABLE_DIMS),
            pose=[wc.TABLE_X_MIN + wc.TABLE_DIMS[0] / 2, -0.10,
                  wc.TABLE_TOP_Z - wc.TABLE_DIMS[2] / 2, 1.0, 0.0, 0.0, 0.0],
        )
        cfg = MotionPlannerCfg.create(
            robot=wc.CUROBO_ROBOT_YML,
            scene_model=Scene(cuboid=[self._table]),
            collision_cache={"cuboid": 8},
            max_goalset=params.motion.yaw_candidates,
            num_ik_seeds=32,
        )
        self.planner = MotionPlanner(cfg)
        self.planner.warmup(enable_graph=False, num_warmup_iterations=2)
        self.joint_names = list(self.planner.joint_names)  # 6 active left-arm joints
        self.interp_dt = float(self.planner.trajopt_solver.config.interpolation_dt)

    # ---- world model -----------------------------------------------------

    def set_world(self, obstacles: list):
        """Rebuild the collision world: table + given cuboids.

        obstacles: list of (name, pos(3), quat_wxyz(4), dims(3)) in base frame.
        """
        cuboids = [self._table]
        for name, pos, quat, dims in obstacles:
            cuboids.append(self._Cuboid(
                name=name, dims=[float(v) for v in dims],
                pose=[float(p) for p in pos] + [float(q) for q in quat],
            ))
        self.planner.update_world(self._Scene(cuboid=cuboids))

    # ---- planning --------------------------------------------------------

    def _joint_state(self, q: np.ndarray):
        from curobo.types import JointState

        t = torch.tensor(np.asarray(q, dtype=np.float32)).view(1, -1).to(self.device)
        return JointState.from_position(t, joint_names=self.joint_names)

    def _goal(self, wrist_pos_list, wrist_quat_list):
        from curobo.types import GoalToolPose

        g = len(wrist_pos_list)
        pos = torch.zeros(1, 1, 1, g, 3, device=self.device)
        quat = torch.zeros(1, 1, 1, g, 4, device=self.device)
        for i, (p, q) in enumerate(zip(wrist_pos_list, wrist_quat_list)):
            pos[0, 0, 0, i] = torch.tensor(np.asarray(p, dtype=np.float32))
            quat[0, 0, 0, i] = torch.tensor(np.asarray(q, dtype=np.float32))
        return GoalToolPose(tool_frames=self.planner.tool_frames, position=pos, quaternion=quat)

    def _resample(self, traj) -> np.ndarray:
        """Interpolated plan -> (N, 6) waypoints at the control rate, slowed
        by time_dilation. Plans may carry the full cspace (locked joints
        included) — select the active arm columns by name."""
        pos = traj.position
        pos = pos.reshape(-1, pos.shape[-1]).detach().cpu().numpy()
        traj_names = list(getattr(traj, "joint_names", None) or self.joint_names)
        if pos.shape[1] != len(self.joint_names):
            cols = [traj_names.index(j) for j in self.joint_names]
            pos = pos[:, cols]
        # the interpolation buffer is padded by repeating the final pose —
        # trim the static tail (keep one terminal waypoint)
        deltas = np.abs(np.diff(pos, axis=0)).max(axis=1)
        moving = np.nonzero(deltas > 1e-6)[0]
        if len(moving):
            pos = pos[: moving[-1] + 2]
        else:
            pos = pos[:1]
        if len(pos) < 2:
            return pos
        dilation = max(self.params.motion.time_dilation, 1e-3)
        control_dt = self.params.episode.sim_dt * self.params.episode.decimation
        t_src = np.arange(len(pos)) * self.interp_dt / dilation
        t_dst = np.arange(0.0, t_src[-1] + 1e-9, control_dt)
        out = np.stack([np.interp(t_dst, t_src, pos[:, j]) for j in range(pos.shape[1])], axis=1)
        return out.astype(np.float32)

    def plan_pick(self, q_start: np.ndarray, tip_grasp_pos: np.ndarray, yaws: list):
        """plan_grasp to a goalset of top-down grasps. Returns dict of
        waypoint segments {approach, grasp, lift} or None."""
        quats = [top_down_wrist_quat(y) for y in yaws]
        goal = self._goal([tip_to_wrist(tip_grasp_pos, q) for q in quats], quats)
        result = self.planner.plan_grasp(
            goal,
            self._joint_state(q_start),
            grasp_approach_axis="y",
            grasp_approach_offset=self.params.motion.hover_offset,
            grasp_approach_in_tool_frame=True,
            grasp_lift_axis="z",
            grasp_lift_offset=self.params.motion.lift_height,
            grasp_lift_in_tool_frame=False,
            plan_approach_to_grasp=True,
            plan_grasp_to_lift=True,
        )
        if result.success is None or not bool(result.success.any()):
            return None
        return {
            "approach": self._resample(result.approach_interpolated_trajectory),
            "grasp": self._resample(result.grasp_interpolated_trajectory),
            "lift": self._resample(result.lift_interpolated_trajectory),
        }

    def plan_move(self, q_start: np.ndarray, tip_pos: np.ndarray, wrist_quat: np.ndarray,
                  disable_finger_collision: bool = False):
        """plan_pose to a single wrist goal; returns waypoints or None."""
        goal = self._goal([tip_to_wrist(tip_pos, wrist_quat)], [wrist_quat])
        contact_links = ["link7l", "link8l", "attached_object"]
        if disable_finger_collision:
            self.planner.disable_link_collision(contact_links)
        try:
            result = self.planner.plan_pose(
                goal, self._joint_state(q_start),
                max_attempts=self.params.motion.max_plan_attempts,
            )
        finally:
            if disable_finger_collision:
                self.planner.enable_link_collision(contact_links)
        if result is None or not bool(result.success.any()):
            return None
        return self._resample(result.get_interpolated_plan())
