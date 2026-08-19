"""Scripted baseline controller: 4-state FSM at 50 Hz over PD position targets.

READY  hold the guard pose (IK-solved once: face at the sweet point, normal
       toward the net).
TRACK  predict the intercept, IK the pre-hit pose (face center at
       p̂* − d·û_out, normal along û_out), drive there on a joint-space
       minimum-jerk profile timed to arrive by t̂* − T_swing. Re-predict every
       tick; replan on > 2 cm drift.
SWING  task-space minimum-jerk from the pre-hit point through p̂*, crossing at
       t̂* with the configured face speed along û_out; each tick is IK-tracked.
FOLLOW decelerate and return to READY.

û_out comes from a small shooting solve on the drag model: the outgoing
velocity from the sweet region to the fixed landing target clearing the net
by >= 0.5 m. v1 commands face velocity along û_out and accepts the resulting
outgoing speed; accuracy is not the goal, clearing the net is.

The PD targets get an inverse-dynamics feedforward (data.qfrc_applied):
tau_ff = clip(M(q) qdd_des + bias + damping*qd_des, torque_limits), so the
servo only corrects deviations from the plan. The real robot needs the same
feedforward from its own model (ledger); the clamp keeps the feedforward
inside the same torque budget as the actuators.
"""

from __future__ import annotations

import mujoco
import numpy as np

import aero
import launcher
import predictor
from baseline.ik import FaceIK
from baseline.minjerk import MinJerk

READY, TRACK, SWING, FOLLOW = "READY", "TRACK", "SWING", "FOLLOW"


class Controller:
    """Drives sim.data.ctrl; call update() once per physics step."""

    def __init__(self, sim, workspace: launcher.Workspace | None = None):
        self.sim = sim
        self.p = sim.params
        self.w = workspace or launcher.load_workspace()
        self.ik = FaceIK(sim, joint_weights=self.p["control"]["ik_joint_weights"])
        # per-joint waypoint step caps (rad per 20 ms tick): distal joints
        # capped near what their motors can actually track (GL40 wrist)
        self._step_cap = np.array(self.p["control"]["swing_step_cap"])
        c = self.p["control"]
        self.dt = sim.model.opt.timestep
        self.steps_per_tick = max(1, int(round(1.0 / (c["hz"] * self.dt))))
        self.tick_dt = self.steps_per_tick * self.dt
        self.t_swing = c["t_swing"]
        self.t_follow = 0.08       # s of follow-through past p̂*
        self.settle = 0.15         # s of hold at the pre-hit pose before SWING
        self.task_kp = 2.0         # jacobian-correction gain during SWING
                                   # (correction step still clamped to 0.15)
        self.prehit_d = c["prehit_offset"]
        self.face_speed = float(np.mean(c["face_speed_range"]))
        self.sweet = np.array(self.p["launcher"]["sweet_point"])
        # PD servo lag compensation: a position servo tracking a ramp lags by
        # v * kv/kp, so lead the commanded target by that much
        self.lead = np.array(c["kv"]) / np.array(c["kp"])
        self._q_lo = sim.model.jnt_range[sim.arm_jids, 0]
        self._q_hi = sim.model.jnt_range[sim.arm_jids, 1]

        # collision-enabled twin of the scene (same qpos layout) for checking
        # planned poses; the runtime model has no arm self-contacts
        self.col_m = launcher.collision_model()
        self.col_d = mujoco.MjData(self.col_m)
        self.col_depth = self.p["workspace"]["self_collision_depth"]

        # guard pose: face at the sweet point, normal toward the net
        self.u_out, self.out_speed, _ = aero.solve_u_out(
            self.sweet, c["landing_target"], self.p)
        self.q_home, e_p, e_o = self._solve_posed(self.sweet, [0, 1, 0])
        self.reset()

    # ------------------------------------------------------------------
    def _collides(self, q, min_dist: float = 0.005) -> bool:
        """True if any self-contact pair is closer than min_dist (negative
        min_dist allows that much penetration). Planning requires 5 mm
        clearance by default; the eval's executed-motion check passes
        min_dist=-self_collision_depth, so tracking error has ~10 mm of
        margin before an episode is flagged.

        Contacts between geoms on the same weld group (racket inside the
        posed gripper fingers; the hanging left arm beside the stand) have
        constant relative pose and are ignored."""
        m, d = self.col_m, self.col_d
        d.qpos[:] = m.qpos0
        d.qpos[self.sim.arm_qadr] = q
        mujoco.mj_kinematics(m, d)
        mujoco.mj_collision(m, d)
        weld = m.body_weldid
        for c in range(d.ncon):
            con = d.contact[c]
            if con.dist >= min_dist:
                continue
            if weld[m.geom_bodyid[con.geom1]] == \
                    weld[m.geom_bodyid[con.geom2]]:
                continue
            return True
        return False

    def _path_clear(self, q_from, q_to, n: int = 8) -> bool:
        """Collision check along the straight joint-space segment (a
        min-jerk profile between two configs traces exactly this segment,
        only reparameterized in time). Endpoints are checked separately."""
        q_from = np.asarray(q_from, dtype=float)
        q_to = np.asarray(q_to, dtype=float)
        # transits get extra clearance (15 mm): they run fast and the servo
        # can dip well below the plan mid-segment
        return not any(self._collides(q_from + s * (q_to - q_from),
                                      min_dist=0.015)
                       for s in np.linspace(0.12, 0.88, n))

    def _fk_face(self, q) -> np.ndarray:
        m, d = self.col_m, self.col_d
        d.qpos[:] = m.qpos0
        d.qpos[self.sim.arm_qadr] = q
        mujoco.mj_kinematics(m, d)
        return d.site_xpos[self.sim.face_sid].copy()

    def _route(self, q_a, q_b, depth: int = 2):
        """Waypoint chain from q_a to q_b with collision-free straight
        segments, or None. A straight joint-space segment cannot go around
        the stand column, so blocked segments recurse through via configs
        borrowed from the workspace cloud: candidates come from several
        points along the task-space line (level and raised), and are tried
        in order of joint-space distance to the segment — a via on the
        wrong IK branch just moves the blockage, so branch-compatible
        configs go first."""
        q_a = np.asarray(q_a, dtype=float)
        q_b = np.asarray(q_b, dtype=float)
        if self._path_clear(q_a, q_b):
            return [q_b]
        if depth == 0:
            return None
        fa, fb = self._fk_face(q_a), self._fk_face(q_b)
        cands = []
        for s in (0.35, 0.5, 0.65):
            x = fa + s * (fb - fa)
            for lift in (0.0, 0.15):
                cands += list(self.w.nearest_configs(x + [0, 0, lift], k=4))
        q_mid = 0.5 * (q_a + q_b)
        cands.sort(key=lambda v: float(np.linalg.norm(v - q_mid)))
        for via in cands[:8]:
            left = self._route(q_a, via, depth - 1)
            if left is None:
                continue
            right = self._route(via, q_b, depth - 1)
            if right is None:
                continue
            return left + right
        return None

    def _solve_posed(self, pos, normal, q_cont=None, ori_weight=0.3):
        """IK with collision-free, natural-branch solutions.

        Tries the continuity seed first (stay on the current branch across
        replans), then the workspace cloud's stored configs near the target
        (each collision-free by construction) as seed + nullspace attractor.
        Returns (q, pos_err, ori_err) of the first candidate that is
        accurate and collision-free, else the best candidate seen.
        """
        best = None
        seeds = []
        if q_cont is not None:
            seeds.append((np.asarray(q_cont, dtype=float), None, 0))
        seeds += [(cfg, cfg, 1) for cfg in self.w.nearest_configs(pos, k=5)]
        for q0, q_ref, restarts in seeds:
            q, e_p, e_o = self.ik.solve(pos, normal, q0, restarts=restarts,
                                        q_ref=q_ref, ori_weight=ori_weight)
            ok = e_p < 0.02 and e_o < 0.35
            col = self._collides(q)
            score = e_p + 0.1 * e_o + (1.0 if col else 0.0)
            if best is None or score < best[0]:
                best = (score, q, e_p, e_o)
            if ok and not col:
                return q, e_p, e_o
        return best[1], best[2], best[3]

    # ------------------------------------------------------------------
    def reset(self):
        self.state = READY
        self.step_count = 0
        self.t = 0.0
        self.q_cmd = self.q_home.copy()
        self.qdd_des = np.zeros_like(self.q_cmd)
        self.qd_des = np.zeros_like(self.q_cmd)
        self._tau_lim = np.array(self.p["arm"]["torque_limits"])
        self._joint_damping = np.array(self.p["arm"]["joint_damping"])
        self.plan = None
        self.plan_t0 = 0.0
        self.swing_plan = None
        self.follow_q = None
        self.follow_i = 0
        self.track_q = None
        self.p_hat = None
        self.t_hat_abs = None      # absolute sim-time of predicted intercept
        self.v_in_hat = None
        self.prehit_pos = None
        self.q_prehit = None
        self.sim.set_arm_targets(self.q_cmd)

    # ------------------------------------------------------------------
    def _predict(self):
        return predictor.predict(self.sim.shuttle_pos, self.sim.shuttle_vel,
                                 self.p, workspace=self.w,
                                 sweet_point=self.sweet)

    def _commit_and_plan(self, pred) -> bool:
        """Choose the intercept: the first candidate (predictor-ranked) whose
        pre-hit pose actually solves — accurate, collision-free, and with
        time to reach it. An intercept whose runway has no natural pose is
        skipped instead of half-attempted. Then route to the pre-hit and lay
        a tick-sampled min-jerk profile over the route. Returns False if no
        candidate is workable."""
        chosen = None
        for j in range(len(pred.cand_p)):
            if pred.cand_t[j] < self.t_swing + self.settle + 0.05:
                continue
            # the pre-hit stays inside the natural-posture workspace box: a
            # runway that pokes out of it has no collision-free natural pose
            prehit = np.clip(pred.cand_p[j] - self.prehit_d * self.u_out,
                             self.w.box_lo + 0.03, self.w.box_hi - 0.03)
            # position accuracy dominates at the pre-hit: the swing chain
            # blends the face into u_out during the stroke
            q_goal, e_p, e_o = self._solve_posed(prehit, self.u_out,
                                                 q_cont=self.q_prehit,
                                                 ori_weight=0.15)
            if e_p >= 0.02 or self._collides(q_goal):
                continue
            # a feasible pre-hit is not enough: the stroke itself must be
            # collision-free, so validate a nominal swing chain here and
            # skip candidates whose stroke would sweep through the body
            _, _, (n_col, _, _) = self._swing_chain(
                prehit, q_goal, self.t_swing + self.t_follow, pred.cand_p[j])
            if n_col == 0:
                chosen = j
                break
        if chosen is None:
            return False
        self.p_hat = pred.cand_p[chosen].copy()
        self.t_hat_abs = self.t + float(pred.cand_t[chosen])
        self.v_in_hat = pred.cand_v[chosen].copy()
        self.prehit_pos = prehit
        self.q_prehit = q_goal
        route = self._route(self.q_cmd, q_goal) or [q_goal]
        arrive_by = self.t_hat_abs - self.t_swing - self.settle
        T = max(arrive_by - self.t, 0.05)
        wps = [np.asarray(self.q_cmd, dtype=float)] + route
        lens = [np.linalg.norm(b - a) for a, b in zip(wps[:-1], wps[1:])]
        total = sum(lens) or 1.0
        n_ticks = max(int(np.ceil(T / self.tick_dt)), 2)
        chain = []
        for a, b, L in zip(wps[:-1], wps[1:], lens):
            nseg = max(int(round(n_ticks * L / total)), 1)
            seg = MinJerk(a, b, nseg * self.tick_dt)
            chain += [seg.at3(i * self.tick_dt)[0] for i in range(nseg)]
        chain.append(np.asarray(q_goal, dtype=float))
        self.track_q = np.array(chain)
        self.track_qd = np.gradient(self.track_q, self.tick_dt, axis=0)
        self.track_qdd = np.gradient(self.track_qd, self.tick_dt, axis=0)
        self.plan_t0 = self.t
        return True

    def _swing_chain(self, start_pos, q_start, T, p_hat):
        """IK-track the swing quintic from start_pos through p_hat with
        terminal velocity face_speed·û_out plus follow-through, trying both
        face sides (joint ranges make them asymmetric). Returns
        (plan, q_wp, (n_col, n_clip, fk_err)) for the better side."""
        end = p_hat + self.face_speed * self.t_follow * self.u_out
        plan = MinJerk(start_pos, end, T, vT=self.face_speed * self.u_out)
        n_wp = int(np.ceil(T / self.tick_dt)) + 1

        def build_chain(side):
            q_wp = np.empty((n_wp, len(q_start)))
            q_seed = np.asarray(q_start, dtype=float)
            fk_err = 0.0
            # branch-stable chain: pin the face side, no nullspace pull, cap
            # waypoint-to-waypoint steps so an IK jump cannot slam the arm.
            # Caps are per joint (control.swing_step_cap): proximal joints get
            # 0.55 rad per 20 ms tick (27.5 rad/s, above any planned stroke),
            # distal joints are capped near their motors' trackable rates.
            for i in range(n_wp):
                x, _, _ = plan.at3(min(i * self.tick_dt, T))
                q_sol, e_p, _ = self.ik.solve(x, self.u_out, q_seed, iters=30,
                                              restarts=0, null_gain=0.0,
                                              face_side=side)
                q_seed = q_seed + np.clip(q_sol - q_seed,
                                          -self._step_cap, self._step_cap)
                q_wp[i] = q_seed
                fk_err += e_p
            # keep planned waypoints away from the joint limits: fast strokes
            # overshoot the plan by ~0.1 rad, which must not cross a limit
            clipped = np.clip(q_wp, self._q_lo + 0.12, self._q_hi - 0.12)
            n_clip = int(np.sum(np.any(clipped != q_wp, axis=1)))
            n_col = sum(self._collides(q) for q in clipped)
            return clipped, (n_col, n_clip, fk_err)

        side = self.ik.face_side(np.asarray(q_start, dtype=float), self.u_out)
        q_wp, score = build_chain(side)
        alt_wp, alt_score = build_chain(-side)
        if alt_score < score:
            q_wp, score = alt_wp, alt_score
        return plan, q_wp, score

    def _plan_swing(self):
        """Task-space quintic: face crosses p̂* at t̂* with face_speed·û_out,
        then follows through past it so the stroke doesn't decelerate into
        the contact.

        The whole waypoint sequence is IK-solved here, once, warm-started
        along the path; execution is open-loop over the precomputed joint
        trajectory (per-tick IK jitter was the dominant tracking noise).
        A stroke that would sweep through the body is not executed: the arm
        holds the pre-hit pose instead (a clean miss beats a dirty hit)."""
        lead = self.p["control"].get("swing_lead", 0.0)
        T = max(self.t_hat_abs - lead - self.t, 0.02) + self.t_follow
        plan, q_wp, (n_col, _, _) = self._swing_chain(
            self.prehit_pos, self.q_cmd, T, self.p_hat)
        if n_col:
            q_wp = np.repeat(np.asarray(self.q_cmd, dtype=float)[None],
                             len(q_wp), axis=0)
        self.swing_q = q_wp
        self.swing_qd = np.gradient(q_wp, self.tick_dt, axis=0)
        self.swing_qdd = np.gradient(self.swing_qd, self.tick_dt, axis=0)
        self.swing_task_plan = plan
        self.plan_t0 = self.t

    # ------------------------------------------------------------------
    def update(self):
        """Advance one physics step: inverse-dynamics feedforward every step,
        control decisions at the 50 Hz tick rate."""
        sim = self.sim
        # tau_ff = clip(M qdd_des + bias, limits): the servo only corrects
        # deviations from the plan
        M = np.zeros((sim.model.nv, sim.model.nv))
        mujoco.mj_fullM(sim.model, sim.data, M)
        M_arm = M[np.ix_(sim.arm_vadr, sim.arm_vadr)]
        # qfrc_bias has Coriolis + gravity but not joint damping; at swing
        # rates (~20 rad/s) damping drag dominates the light distal links
        tau = (M_arm @ self.qdd_des + sim.data.qfrc_bias[sim.arm_vadr]
               + self._joint_damping * self.qd_des)
        lim = self._tau_lim
        sim.data.qfrc_applied[sim.arm_vadr] = np.clip(tau, -lim, lim)

        if self.step_count % self.steps_per_tick == 0:
            self._tick()
        self.step_count += 1
        self.t = self.step_count * self.dt

    def _tick(self):
        sim = self.sim
        qd_des = np.zeros_like(self.q_cmd)
        qdd_des = np.zeros_like(self.q_cmd)
        if self.state == READY:
            pred = self._predict()
            if pred.ok and self._commit_and_plan(pred):
                self.state = TRACK
            else:
                self.q_cmd = self.q_home.copy()
        elif self.state == TRACK:
            pred = self._predict()
            if pred.ok:
                # follow the committed crossing: update p̂*/t̂* from the
                # fresh trajectory sample nearest to it, and re-commit only
                # if that crossing itself has drifted
                d = np.linalg.norm(pred.cand_p - self.p_hat, axis=1)
                j = int(np.argmin(d))
                if d[j] > 0.02:
                    self._commit_and_plan(pred)
                else:
                    self.p_hat = pred.cand_p[j].copy()
                    self.t_hat_abs = self.t + float(pred.cand_t[j])
                    self.v_in_hat = pred.cand_v[j].copy()
            if self.t >= self.t_hat_abs - self.t_swing:
                self.state = SWING
                self._plan_swing()
            else:
                i = int(round((self.t - self.plan_t0) / self.tick_dt))
                if i < len(self.track_q):
                    self.q_cmd = self.track_q[i].copy()
                    qd_des = self.track_qd[i]
                    qdd_des = self.track_qdd[i]
                else:
                    self.q_cmd = self.q_prehit.copy()
        if self.state == SWING:
            i = int(round((self.t - self.plan_t0) / self.tick_dt))
            if i >= len(self.swing_q):
                # retreat along the executed stroke (known collision-free)
                # at 1/3 speed, then a checked transit back to guard
                self.state = FOLLOW
                rev = self.swing_q[::-1]
                idx = np.minimum((np.arange(3 * len(rev)) / 3).astype(int),
                                 len(rev) - 1)
                self.follow_q = rev[idx]
                self.follow_qd = np.gradient(self.follow_q, self.tick_dt,
                                             axis=0)
                self.follow_i = 0
                self.plan = None
                self.plan_t0 = self.t
            else:
                # open-loop chain + small jacobian correction toward the
                # planned face position (kills pose-dependent transient drift)
                q = self.swing_q[i].copy()
                x_plan, _, _ = self.swing_task_plan.at3(i * self.tick_dt)
                e = x_plan - self.sim.face_pos
                e_n = np.linalg.norm(e)
                if 0.005 < e_n < 0.3:
                    dq = self.ik.correction(q, e)
                    q += np.clip(self.task_kp * dq, -0.15, 0.15)
                self.q_cmd = q
                qd_des = self.swing_qd[i]
                qdd_des = self.swing_qdd[i]
        elif self.state == FOLLOW:
            if self.follow_i < len(self.follow_q):
                self.q_cmd = self.follow_q[self.follow_i].copy()
                qd_des = self.follow_qd[self.follow_i]
                self.follow_i += 1
            elif self.plan is None:
                # transit back to guard only once the segment is clear;
                # until then hold the retreat pose and retry each tick
                if self._path_clear(self.q_cmd, self.q_home):
                    self.plan = MinJerk(self.q_cmd, self.q_home, 0.8)
                    self.plan_t0 = self.t
            else:
                self.q_cmd, qd_des, qdd_des = \
                    self.plan.at3(self.t - self.plan_t0)
                if self.t - self.plan_t0 > 0.9:
                    self.state = READY
        # feedforward accel, capped so plan discontinuities don't slam the arm
        self.qdd_des = np.clip(qdd_des, -600.0, 600.0)
        self.qd_des = qd_des
        # lead the servo target to cancel the tracking lag kv/kp * qdot;
        # never command a target at or beyond a joint limit
        target = np.clip(self.q_cmd + self.lead * qd_des,
                         self._q_lo + 0.02, self._q_hi - 0.02)
        sim.set_arm_targets(target)
