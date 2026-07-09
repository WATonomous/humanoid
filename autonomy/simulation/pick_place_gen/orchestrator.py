"""Episode orchestrator: phase machine over cuRobo plans -> joint actions.

Consumes ground-truth state each control step and emits an 8-dim action
[6 arm joint targets + joint7l + joint8l]. Phases:

  PLAN_PICK -> APPROACH -> DESCEND -> GRASP -> LIFT -> (verify grasp)
  -> TRANSIT (optional random via-point) -> PLACE_DESCEND -> RELEASE
  -> RETRACT -> DONE  (or FAILED(reason) at any point)

Waypoints stream one per control step; per-step Gaussian noise on the arm
targets (params.noise) forces the PD controller to correct — the recovery
signal a clean scripted expert otherwise never produces.
"""
import math

import numpy as np

import wato_constants as wc
from curobo_expert import CuRoboExpert, top_down_wrist_quat
from task_params import PickPlaceTaskParams

OPEN = np.array([wc.GRIPPER_OPEN["joint7l"], wc.GRIPPER_OPEN["joint8l"]], dtype=np.float32)
CLOSED = np.array([wc.GRIPPER_CLOSED["joint7l"], wc.GRIPPER_CLOSED["joint8l"]], dtype=np.float32)


class Orchestrator:
    def __init__(self, expert: CuRoboExpert, params: PickPlaceTaskParams,
                 rng: np.random.Generator):
        self.expert = expert
        self.params = params
        self.rng = rng
        self.control_dt = params.episode.sim_dt * params.episode.decimation
        self.phase_timeout = int(params.episode.phase_timeout_s / self.control_dt)

    # ---- episode lifecycle -------------------------------------------------

    def start_episode(self, state: dict) -> bool:
        """Plan the pick. state: object_pos/quat/size, place_pos, q_arm.
        Returns False (failure_reason set) if planning fails."""
        p = self.params
        self.phase = "APPROACH"
        self.failure_reason = None
        self.steps_in_phase = 0
        self.gripper = OPEN.copy()
        self._track_err_steps = 0

        obj_pos = np.asarray(state["object_pos"])
        self.obj_size = np.asarray(state["object_size"])
        self.place_pos = np.asarray(state["place_pos"])
        self.support_z = float(state["support_top_z"])

        # grasp yaw: object yaw snapped to 90 deg + jitter, plus goalset spread
        obj_yaw = state["object_yaw"]
        base_yaw = round(obj_yaw / (math.pi / 2)) * (math.pi / 2)
        base_yaw += math.radians(self.rng.uniform(-p.noise.grasp_yaw_jitter_deg,
                                                  p.noise.grasp_yaw_jitter_deg))
        n = p.motion.yaw_candidates
        yaws = [base_yaw + i * math.pi / 2 for i in range(4)]
        yaws += [base_yaw + math.pi / 4 + i * math.pi / 2 for i in range(max(n - 4, 0))]
        yaws = yaws[:n]

        tip_grasp = obj_pos.copy()
        # pads hang FINGER_PAD_BELOW_TIP below the tip-center reference; keep
        # them clear of the support surface while gripping the object's side
        obj_top = obj_pos[2] + self.obj_size[2] / 2
        tip_grasp[2] = max(
            obj_top - p.motion.grasp_tip_depth,
            wc.TABLE_TOP_Z + wc.FINGER_PAD_BELOW_TIP + 0.004,
        )
        if p.noise.enabled:
            tip_grasp[:2] += self.rng.uniform(-0.003, 0.003, 2)

        obstacles = [("object", state["object_pos"], state["object_quat"], self.obj_size)]
        if state.get("place_object_pos") is not None:
            obstacles.append(("place_object", state["place_object_pos"],
                              state["place_object_quat"], state["place_object_size"]))
        self._transit_obstacles = obstacles[1:]  # object is in-hand during transit
        self.expert.set_world(obstacles)

        segments = self.expert.plan_pick(state["q_arm"], tip_grasp, yaws)
        if segments is None:
            self.failure_reason = "plan_pick_failed"
            self.phase = "FAILED"
            return False
        self._segments = segments
        self._queue = list(segments["approach"])
        self._grasp_yaw = base_yaw
        self._hold = None
        self._to("APPROACH")
        return True

    # ---- per-step ------------------------------------------------------------

    def step(self, state: dict) -> np.ndarray:
        """Advance one control step; returns the 8-dim action."""
        p = self.params
        self.steps_in_phase += 1
        budget = getattr(self, "_phase_budget", self.phase_timeout)
        if self.phase not in ("DONE", "FAILED") and self.steps_in_phase > budget:
            self._fail(f"timeout_{self.phase.lower()}")

        handler = getattr(self, f"_phase_{self.phase.lower()}", None)
        if handler is not None:
            handler(state)

        arm = self._current_arm_target(state)
        if p.noise.enabled and self.phase not in ("DONE", "FAILED"):
            arm = arm + self.rng.normal(0.0, math.radians(p.noise.joint_noise_std_deg), 6)
        self._check_tracking(state, arm)
        return np.concatenate([arm, self.gripper]).astype(np.float32)

    @property
    def done(self) -> bool:
        return self.phase in ("DONE", "FAILED")

    # ---- phase handlers --------------------------------------------------

    def _phase_approach(self, state):
        if not self._queue:
            self._queue = list(self._segments["grasp"])
            self._to("DESCEND")

    def _phase_descend(self, state):
        if not self._queue:
            # keep holding the final COMMANDED waypoint (not the measured pose,
            # which lags behind and would freeze tracking error into the grasp)
            self._close_ticks = 0
            self._to("GRASP")

    def _phase_grasp(self, state):
        ramp = 10
        settle = self.params.success.settle_steps // 2 + 5
        self._close_ticks += 1
        alpha = min(self._close_ticks / ramp, 1.0)
        self.gripper = (1 - alpha) * OPEN + alpha * CLOSED
        if self._close_ticks >= ramp + settle:
            self._queue = list(self._segments["lift"])
            self._to("LIFT")

    def _phase_lift(self, state):
        if not self._queue:
            # grasp verification: the object must have come up with the hand
            if state["object_pos"][2] < wc.TABLE_TOP_Z + 0.05:
                self._fail("grasp_failed")
                return
            # measured object-center height relative to the tip while held —
            # used to compute the place descend height exactly
            self._held_obj_dz = float(state["object_pos"][2] - state["tip_pos"][2])
            self._plan_transit(state)

    def _phase_transit(self, state):
        if not self._queue:
            self._plan_place_descend(state)

    def _phase_place_descend(self, state):
        if not self._queue:
            self._open_ticks = 0
            self._to("RELEASE")

    def _phase_release(self, state):
        ramp = 8
        self._open_ticks += 1
        alpha = min(self._open_ticks / ramp, 1.0)
        self.gripper = (1 - alpha) * CLOSED + alpha * OPEN
        if self._open_ticks >= ramp + 10:
            self._plan_retract(state)

    def _phase_retract(self, state):
        if not self._queue:
            self._to("DONE")

    # ---- transit / place planning -----------------------------------------

    def _place_quat(self):
        return top_down_wrist_quat(getattr(self, "_place_yaw", self._grasp_yaw))

    def _tip_z_for_place(self):
        # place so the held object's bottom lands at support + clearance,
        # using the object-vs-tip height measured after LIFT
        p = self.params
        held_dz = getattr(self, "_held_obj_dz", -p.motion.grasp_tip_depth)
        return (self.support_z + p.motion.place_clearance
                + self.obj_size[2] / 2 - held_dz)

    def _place_yaw_candidates(self):
        """The cube is yaw-symmetric — any 90-degree-equivalent place yaw works.
        Try the grasp yaw first, then rotations of it."""
        base = self._grasp_yaw
        return [base, base + math.pi / 2, base - math.pi / 2, base + math.pi / 4,
                base - math.pi / 4, base + math.pi]

    def _plan_transit(self, state):
        p = self.params
        self.expert.set_world(self._transit_obstacles)
        q0 = np.asarray(state["q_arm"])
        for yaw in self._place_yaw_candidates():
            quat = top_down_wrist_quat(yaw)
            hover_tip = np.array([self.place_pos[0], self.place_pos[1],
                                  self._tip_z_for_place() + p.motion.place_hover_offset])
            q = q0
            waypoints = []
            if p.noise.enabled and self.rng.uniform() < p.noise.via_point_prob:
                cur_tip = np.asarray(state["tip_pos"])
                mid = (cur_tip + hover_tip) / 2
                mid[:2] += self.rng.uniform(-p.noise.via_point_lateral, p.noise.via_point_lateral, 2)
                mid[2] += self.rng.uniform(0.0, p.noise.via_point_vertical)
                mid[2] = float(np.clip(mid[2], wc.TABLE_TOP_Z + 0.12, wc.TABLE_TOP_Z + 0.15))
                seg1 = self.expert.plan_move(q, mid, quat)
                if seg1 is not None:
                    waypoints += list(seg1)
                    q = seg1[-1]
            seg2 = self.expert.plan_move(q, hover_tip, quat)
            if seg2 is not None:
                self._place_yaw = yaw
                self._queue = waypoints + list(seg2)
                self._to("TRANSIT")
                return
        self._fail("plan_transit_failed")

    def _plan_place_descend(self, state):
        tip = np.array([self.place_pos[0], self.place_pos[1], self._tip_z_for_place()])
        seg = self.expert.plan_move(np.asarray(state["q_arm"]), tip, self._place_quat(),
                                    disable_finger_collision=True)
        if seg is None:
            self._fail("plan_place_failed")
            return
        self._queue = list(seg)
        self._to("PLACE_DESCEND")

    def _plan_retract(self, state):
        p = self.params
        tip = np.array([self.place_pos[0], self.place_pos[1],
                        self._tip_z_for_place() + p.motion.place_hover_offset])
        seg = self.expert.plan_move(np.asarray(state["q_arm"]), tip, self._place_quat(),
                                    disable_finger_collision=True)
        if seg is None:
            # retract failure is benign: hold pose and finish
            self._queue = []
            self._to("DONE")
            return
        self._queue = list(seg)
        self._to("RETRACT")

    # ---- helpers -----------------------------------------------------------

    def _current_arm_target(self, state) -> np.ndarray:
        if self._queue:
            wp = self._queue.pop(0)
            self._hold = np.asarray(wp, dtype=np.float32)
        if self._hold is None:
            self._hold = np.asarray(state["q_arm"], dtype=np.float32)
        return self._hold.copy()

    def _check_tracking(self, state, arm_target):
        err = np.abs(np.asarray(state["q_arm"]) - arm_target).max()
        self._track_err_steps = self._track_err_steps + 1 if err > 0.5 else 0
        if self._track_err_steps > 50:
            self._fail("tracking_lost")

    def _to(self, phase: str):
        self.phase = phase
        self.steps_in_phase = 0
        # streaming phases: watchdog scales with the plan length
        self._phase_budget = max(self.phase_timeout, int(len(self._queue) * 1.5) + 100)

    def _fail(self, reason: str):
        self.failure_reason = reason
        self.phase = "FAILED"
