"""Interactive sim harness (phases 2-5): viewer with overlays and live tuning.

Usage: uv run python scripts/sim_lab.py [--episode N]

Hotkeys (viewer window):
  space  pause / resume
  .      single physics step while paused
  [ ]    time dilation down / up (0.1x .. 1x)
  n      fire the next launcher episode
  r      re-read params.yaml into the running model (k, solref, gains)
  -/=    face-pair dampratio down / up (bounce tuning)
  ,/'    drag k down / up
  v      toggle visual meshes (group 2)
  b      toggle collision primitives (group 3)
  c      cycle camera preset (side-on at net / top-down / track shuttle / track racket)
  m      toggle contact point + force rendering
  x      x-ray nudge mode help (prints geom nudge keys)

Overlays: reference RK4 trajectory (ghost polyline), predicted intercept p̂*
(sphere), W point cloud if launcher artifacts exist, landing target, pre-hit
pose marker. Status and last-episode metrics print to the terminal.
"""

import argparse
import os
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import aero
import mjsim

try:
    import launcher as launcher_mod
except Exception:
    launcher_mod = None
try:
    import predictor as predictor_mod
except Exception:
    predictor_mod = None
try:
    from baseline import fsm as fsm_mod
except Exception:
    fsm_mod = None

GHOST_RGBA = np.array([0.4, 0.7, 1.0, 0.5])
PRED_RGBA = np.array([1.0, 0.2, 0.2, 0.8])
W_RGBA = np.array([0.3, 0.9, 0.4, 0.12])
PREHIT_RGBA = np.array([0.9, 0.9, 0.1, 0.8])


class SimLab:
    def __init__(self):
        self.sim = mjsim.load()
        self.paused = False
        self.do_single = False
        self.dilation = 1.0
        self.cam_idx = 0
        self.show_contacts = True
        self.episode = None       # (p0, v0, ref_traj)
        self.metrics = {}
        self.w_cloud = self._load_w_cloud()
        self.controller = fsm_mod.Controller(self.sim) if fsm_mod else None
        self.prehit_marker = None
        self.pred_point = None

    def _load_w_cloud(self):
        path = os.path.join(os.path.dirname(mjsim.SCENE_PATH), "workspace_W.npz")
        if os.path.exists(path):
            pts = np.load(path)["points"]
            step = max(1, len(pts) // 400)
            return pts[::step]
        return None

    # ------------------------------------------------------------------
    def next_episode(self):
        sim = self.sim
        if launcher_mod is not None and launcher_mod.workspace_exists():
            spec = launcher_mod.sample_specs(1)[0]
            p0, v0, traj = spec.p0, spec.v0, spec.ref_traj
        else:
            # fallback: a lofted feed that passes ~13 cm from the home face
            # pose at t ~ 1.06 s arriving at ~5 m/s (found by forward scan)
            ang = np.radians(35.0)
            p0 = np.array([sim.face_pos[0], 4.5, 1.5])
            v0 = 13.5 * np.array([0.0, -np.cos(ang), np.sin(ang)])
            t, P, V = aero.simulate(p0, v0, sim.k, t_max=2.5)
            traj = (t, P, V)
        q_home = np.zeros(6)
        sim.reset(p0=p0, v0=v0, arm_q=q_home)
        if self.controller:
            self.controller.reset()
        self.episode = (p0, v0, traj)
        self.metrics = {"min_dist": np.inf, "contact": False,
                        "contact_speed": 0.0, "landing": None}
        print(f"episode: launch p0={np.round(p0, 2)} |v0|={np.linalg.norm(v0):.1f}")

    def update_metrics(self):
        sim = self.sim
        d = float(np.linalg.norm(sim.face_pos - sim.shuttle_pos))
        self.metrics["min_dist"] = min(self.metrics["min_dist"], d)
        if sim.face_contact() and not self.metrics["contact"]:
            self.metrics["contact"] = True
            rel = sim.shuttle_vel - sim.face_vel
            self.metrics["contact_speed"] = float(np.linalg.norm(rel))
        p = sim.shuttle_pos
        if p[2] < 0.05 and self.metrics["landing"] is None:
            self.metrics["landing"] = p[:2].copy()
            lt = np.array(sim.params["control"]["landing_target"])
            err = float(np.linalg.norm(p[:2] - lt))
            m = self.metrics
            print(f"  landed: min_dist={m['min_dist']:.3f} contact={m['contact']} "
                  f"contact_speed={m['contact_speed']:.2f} landing_err={err:.2f}")

    # ------------------------------------------------------------------
    def key_cb(self, keycode):
        sim = self.sim
        c = chr(keycode) if 32 <= keycode < 127 else None
        if keycode == 32:
            self.paused = not self.paused
        elif c == ".":
            self.do_single = True
        elif c == "[":
            self.dilation = max(0.1, self.dilation / 2)
            print(f"time dilation {self.dilation:.2f}x")
        elif c == "]":
            self.dilation = min(1.0, self.dilation * 2)
            print(f"time dilation {self.dilation:.2f}x")
        elif c in ("n", "N"):
            self.next_episode()
        elif c in ("r", "R"):
            self.reload_params()
        elif c == "-":
            self.nudge_dampratio(-0.02)
        elif c == "=":
            self.nudge_dampratio(+0.02)
        elif c == ",":
            sim.k = max(0.05, sim.k - 0.005)
            print(f"k = {sim.k:.3f} (v_t = {np.sqrt(9.81/sim.k):.2f})")
        elif c == "'":
            sim.k += 0.005
            print(f"k = {sim.k:.3f} (v_t = {np.sqrt(9.81/sim.k):.2f})")
        elif c in ("v", "V"):
            self.viewer.opt.geomgroup[2] ^= 1
        elif c in ("b", "B"):
            self.viewer.opt.geomgroup[3] ^= 1
        elif c in ("c", "C"):
            self.cam_idx = (self.cam_idx + 1) % 4
            self.apply_camera()
        elif c in ("m", "M"):
            self.show_contacts = not self.show_contacts
            self.apply_vis_flags()

    def nudge_dampratio(self, delta):
        m = self.sim.model
        # face pair is the one whose geoms include racket_face
        for i in range(m.npair):
            if self.sim.face_gid in (m.pair_geom1[i], m.pair_geom2[i]):
                m.pair_solref[i, 1] = max(0.02, m.pair_solref[i, 1] + delta)
                print(f"face pair solref = {m.pair_solref[i]}")

    def reload_params(self):
        sim = self.sim
        sim.params = aero.load_params()
        sim.k = sim.params["shuttle"]["k"]
        m = sim.model
        c = sim.params["contact"]
        for i in range(m.npair):
            if sim.face_gid in (m.pair_geom1[i], m.pair_geom2[i]):
                m.pair_solref[i, :2] = c["face_solref"]
                m.pair_solimp[i, :3] = c["face_solimp"]
        print("params.yaml reloaded")

    def apply_camera(self):
        cam = self.viewer.cam
        cam.trackbodyid = -1
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        if self.cam_idx == 0:      # side-on at the net
            cam.lookat[:] = [0, 0, 1.2]
            cam.azimuth, cam.elevation, cam.distance = 90, -10, 6.0
        elif self.cam_idx == 1:    # top-down court
            cam.lookat[:] = [0, 0, 0]
            cam.azimuth, cam.elevation, cam.distance = 90, -89, 12.0
        elif self.cam_idx == 2:    # track shuttle
            cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            cam.trackbodyid = self.sim.shuttle_bid
            cam.distance = 3.0
        else:                      # track racket
            cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            cam.trackbodyid = self.sim.model.body("racket").id
            cam.distance = 2.0
        print(f"camera preset {self.cam_idx}")

    def apply_vis_flags(self):
        opt = self.viewer.opt
        on = 1 if self.show_contacts else 0
        opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = on
        opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = on

    # ------------------------------------------------------------------
    def draw_overlays(self):
        scn = self.viewer.user_scn
        scn.ngeom = 0
        idx = 0

        def add_sphere(pos, r, rgba):
            nonlocal idx
            if idx >= scn.maxgeom:
                return
            mujoco.mjv_initGeom(scn.geoms[idx], mujoco.mjtGeom.mjGEOM_SPHERE,
                                np.array([r, 0, 0]), np.asarray(pos, dtype=float),
                                np.eye(3).ravel(), rgba.astype(np.float32))
            idx += 1

        def add_segment(a, b, rgba, width=0.002):
            nonlocal idx
            if idx >= scn.maxgeom:
                return
            mujoco.mjv_initGeom(scn.geoms[idx], mujoco.mjtGeom.mjGEOM_CAPSULE,
                                np.zeros(3), np.zeros(3), np.eye(3).ravel(),
                                rgba.astype(np.float32))
            mujoco.mjv_connector(scn.geoms[idx], mujoco.mjtGeom.mjGEOM_CAPSULE,
                                 width, np.asarray(a, float), np.asarray(b, float))
            idx += 1

        if self.episode is not None:
            t, P, V = self.episode[2]
            step = max(1, len(P) // 60)
            for i in range(0, len(P) - step, step):
                add_segment(P[i], P[i + step], GHOST_RGBA)
        if self.w_cloud is not None:
            for pt in self.w_cloud:
                add_sphere(pt, 0.01, W_RGBA)
        if self.pred_point is not None:
            add_sphere(self.pred_point, 0.03, PRED_RGBA)
        if self.prehit_marker is not None:
            add_sphere(self.prehit_marker, 0.025, PREHIT_RGBA)
        scn.ngeom = idx

    # ------------------------------------------------------------------
    def run(self):
        sim = self.sim
        with mujoco.viewer.launch_passive(sim.model, sim.data,
                                          key_callback=self.key_cb) as viewer:
            self.viewer = viewer
            self.apply_vis_flags()
            self.apply_camera()
            self.next_episode()
            while viewer.is_running():
                t0 = time.perf_counter()
                if not self.paused or self.do_single:
                    self.do_single = False
                    if self.controller and self.episode is not None:
                        self.controller.update()
                        self.pred_point = getattr(self.controller, "p_hat", None)
                        self.prehit_marker = getattr(self.controller, "prehit_pos", None)
                    elif predictor_mod is not None and self.episode is not None:
                        pred = predictor_mod.predict(sim.shuttle_pos,
                                                     sim.shuttle_vel, sim.params)
                        self.pred_point = pred.p_star if pred.ok else None
                    sim.step()
                    self.update_metrics()
                self.draw_overlays()
                viewer.sync()
                elapsed = time.perf_counter() - t0
                budget = sim.model.opt.timestep / self.dilation
                if elapsed < budget:
                    time.sleep(budget - elapsed)


def main():
    ap = argparse.ArgumentParser()
    ap.parse_args()
    SimLab().run()


if __name__ == "__main__":
    main()
