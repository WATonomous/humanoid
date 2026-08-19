"""Damped-least-squares IK on the mujoco jacobian for the racket face.

Solves 5 constraints: face-center position (3) + face normal direction (2,
the component of normal error orthogonal to the normal itself). Iterates on a
scratch MjData so the sim state is untouched. Joint-limit avoidance: clamp to
limits each iteration plus a small nullspace pull toward mid-range.
"""

from __future__ import annotations

import mujoco
import numpy as np


class FaceIK:
    def __init__(self, sim, damping: float = 0.05, limit_margin: float = 0.10,
                 joint_weights=None):
        self.sim = sim
        self.model = sim.model
        self.scratch = mujoco.MjData(self.model)
        self.damping = damping
        self.lo = self.model.jnt_range[sim.arm_jids, 0] + limit_margin
        self.hi = self.model.jnt_range[sim.arm_jids, 1] - limit_margin
        self.mid = (self.lo + self.hi) / 2.0
        # weighted-joint DLS: dq = D J^T (J D J^T + lambda^2 I)^-1 e with
        # D = diag(1/w^2), so a joint with weight w moves 1/w^2 as readily.
        # Used to spare weak distal motors (the GL40 wrist) and let the
        # shoulder/elbow drive strokes.
        w = np.ones(len(sim.arm_jids)) if joint_weights is None \
            else np.asarray(joint_weights, dtype=float)
        self.D = 1.0 / w**2

    def solve(self, target_pos, target_normal, q_init, iters: int = 60,
              tol: float = 1e-4, ori_weight: float = 0.3,
              null_gain: float = 0.05, restarts: int = 4, seed: int = 0,
              face_side: float | None = None, q_ref=None):
        """Return (q, pos_err, ori_err). target_normal may be satisfied by
        either face side (the sign with smaller error is used, unless
        face_side pins it to +1/-1 for branch-stable waypoint chains).
        Retries from random configs if the first descent stalls.

        q_ref anchors the nullspace pull (default: joint mid-range) and the
        restart distribution: pass a known-good config (e.g. a workspace
        cloud config near the target) to keep the solution on that branch."""
        rng = np.random.default_rng(seed)
        best = None
        q0 = np.asarray(q_init, dtype=float).copy()
        for attempt in range(restarts + 1):
            q, e_p, e_o = self._descend(target_pos, target_normal, q0, iters,
                                        tol, ori_weight, null_gain, face_side,
                                        q_ref)
            if best is None or (e_p + 0.1 * e_o) < (best[1] + 0.1 * best[2]):
                best = (q, e_p, e_o)
            if best[1] < 0.005 and best[2] < 0.1:
                break
            if q_ref is not None:
                q0 = np.clip(np.asarray(q_ref, dtype=float)
                             + rng.normal(0, 0.3, len(q0)), self.lo, self.hi)
            else:
                q0 = rng.uniform(self.lo, self.hi) * 0.5   # biased toward mid
        return best

    def correction(self, q, e_pos) -> np.ndarray:
        """One DLS position-only step at config q toward face error e_pos."""
        m, d = self.model, self.scratch
        sim = self.sim
        d.qpos[:] = m.qpos0
        d.qpos[sim.arm_qadr] = q
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        jacp = np.zeros((3, m.nv))
        mujoco.mj_jacSite(m, d, jacp, None, sim.face_sid)
        J = jacp[:, sim.arm_vadr]
        A = J * self.D[None, :]
        JJt = A @ J.T + self.damping**2 * np.eye(3)
        return A.T @ np.linalg.solve(JJt, np.asarray(e_pos, dtype=float))

    def face_side(self, q, target_normal) -> float:
        """Which face side (+1/-1) currently points along target_normal."""
        m, d = self.model, self.scratch
        d.qpos[:] = m.qpos0
        d.qpos[self.sim.arm_qadr] = q
        mujoco.mj_kinematics(m, d)
        n = d.site_xmat[self.sim.face_sid].reshape(3, 3)[:, 2]
        return 1.0 if n @ np.asarray(target_normal, dtype=float) >= 0 else -1.0

    def _descend(self, target_pos, target_normal, q_init, iters, tol,
                 ori_weight, null_gain, face_side=None, q_ref=None):
        m, d = self.model, self.scratch
        sim = self.sim
        target_pos = np.asarray(target_pos, dtype=float)
        n_des = np.asarray(target_normal, dtype=float)
        n_des = n_des / np.linalg.norm(n_des)
        attractor = self.mid if q_ref is None \
            else np.asarray(q_ref, dtype=float)
        q = np.clip(np.asarray(q_init, dtype=float).copy(), self.lo, self.hi)

        jacp = np.zeros((3, m.nv))
        jacr = np.zeros((3, m.nv))
        pos_err_n = ori_err_n = np.inf
        for _ in range(iters):
            d.qpos[:] = m.qpos0    # valid free-joint quaternion for the shuttle
            d.qpos[sim.arm_qadr] = q
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            face = d.site_xpos[sim.face_sid]
            R = d.site_xmat[sim.face_sid].reshape(3, 3)
            n_cur = R[:, 2].copy()
            if face_side is not None:
                flip = face_side
                n_cur = flip * n_cur
            elif n_cur @ n_des < 0:    # either face side counts
                n_cur = -n_cur
                flip = -1.0
            else:
                flip = 1.0
            e_pos = target_pos - face
            e_n = n_des - n_cur        # normal-vector error, pairs with Jn
            pos_err_n = float(np.linalg.norm(e_pos))
            ori_err_n = float(np.linalg.norm(np.cross(n_cur, n_des)))
            if pos_err_n < tol and ori_err_n < 0.02:
                break
            mujoco.mj_jacSite(m, d, jacp, jacr, sim.face_sid)
            Jp = jacp[:, sim.arm_vadr]
            Jr = jacr[:, sim.arm_vadr]
            # dn/dq_j = omega_j x n  (n rotates with the body)
            Jn = flip * np.cross(Jr.T, R[:, 2]).T   # (3, 6)
            J = np.vstack([Jp, ori_weight * Jn])
            e = np.concatenate([e_pos, ori_weight * e_n])
            A = J * self.D[None, :]           # J D, D = diag(1/w^2)
            JJt = A @ J.T + self.damping**2 * np.eye(J.shape[0])
            dq = A.T @ np.linalg.solve(JJt, e)
            # nullspace pull toward the attractor (mid-range by default, or
            # a known-good branch config via q_ref)
            JtJinvJt = A.T @ np.linalg.solve(JJt, J)
            N = np.eye(len(q)) - JtJinvJt
            dq += null_gain * (N @ (attractor - q))
            step = np.linalg.norm(dq)
            if step > 0.5:
                dq *= 0.5 / step
            q = np.clip(q + dq, self.lo, self.hi)
        return q, pos_err_n, ori_err_n
