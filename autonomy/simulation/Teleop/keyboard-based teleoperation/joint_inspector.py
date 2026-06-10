"""
joint_inspector.py
==================
Interactively inspect every joint (then every link) of the WATO bimanual arm.

PHASE 1 — JOINTS
  • The script sweeps the currently-selected joint back and forth with a
    smooth sinusoidal motion so you can clearly see its range of motion.
  • All other joints stay at their default (zero) position.
  • Press  P  to advance to the next joint.
  • After all joints have been visited, Phase 2 begins automatically.

PHASE 2 — LINKS (body-frame visualisation)
  • A coloured axis-frame marker is placed at each link's world position
    so you can see exactly where each link lives in space.
  • Press  P  to cycle the highlighted marker to the next link.
  • Press  P  on the last link to exit.

Run (non-headless so you can press P):
    python joint_inspector.py
"""

# ── AppLauncher must be the very first Isaac import ──────────────────────────
import argparse
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Joint & Link Inspector")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = False          # GUI is required to press P

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ── All other imports AFTER SimulationApp is alive ───────────────────────────
import math
import torch
import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.markers import VisualizationMarkers
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass

# omni.ui is available only in GUI mode
import omni.ui as ui

# ── WATO bimanual arm config (same URDF as task_space_test) ──────────────────
_BIMANUAL_URDF_PATH = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "..", "..",
        "Humanoid_Wato", "wato_bimanual_arm", "urdf", "armDouble.SLDASM.urdf",
    )
)

WATO_BIMANUAL_ARM_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=_BIMANUAL_URDF_PATH,
        fix_base=True,
        joint_drive=sim_utils.UrdfFileCfg.JointDriveCfg(
            drive_type="force",
            target_type="position",
            gains=sim_utils.UrdfFileCfg.JointDriveCfg.PDGainsCfg(
                stiffness=400.0,
                damping=40.0,
            ),
        ),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        joint_pos={
            "joint1": 0.0, "joint2": 0.0, "joint3": 0.0,
            "joint4": 0.0, "joint5": 0.0, "joint6": 0.0,
            "joint7": 0.0, "joint8": 0.0,
            "joint1L": 0.0, "joint2l": 0.0, "joint3l": 0.0,
            "joint4l": 0.0, "joint5l": 0.0, "joint6l": 0.0,
            "joint7l": 0.0, "joint8l": 0.0,
        },
    ),
    actuators={
        "right_arm": ImplicitActuatorCfg(
            joint_names_expr=["joint[1-6]"],
            stiffness=400.0, damping=40.0, velocity_limit_sim=3.0,
        ),
        "right_gripper": ImplicitActuatorCfg(
            joint_names_expr=["joint[7-8]"],
            stiffness=200.0, damping=20.0, velocity_limit_sim=1.0,
        ),
        "left_arm": ImplicitActuatorCfg(
            joint_names_expr=["joint[1-6]L", "joint[2-6]l"],
            stiffness=400.0, damping=40.0, velocity_limit_sim=3.0,
        ),
        "left_gripper": ImplicitActuatorCfg(
            joint_names_expr=["joint[7-8]l"],
            stiffness=200.0, damping=20.0, velocity_limit_sim=1.0,
        ),
    },
)


# ── Scene config ─────────────────────────────────────────────────────────────
@configclass
class InspectorSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )
    robot = WATO_BIMANUAL_ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


# ── Sweep amplitude (radians) per joint ──────────────────────────────────────
# Tune these if a joint hits a hard limit and looks wrong.
# Format: { joint_name_substring : amplitude_rad }
JOINT_AMPLITUDES = {
    "joint1":  0.8,
    "joint2":  0.8,
    "joint3":  0.8,
    "joint4":  0.8,
    "joint5":  0.8,
    "joint6":  0.8,
    "joint7":  0.03,   # gripper finger (metres or radians depending on URDF)
    "joint8":  0.03,
    "joint1L": 0.8,
    "joint2l": 0.8,
    "joint3l": 0.8,
    "joint4l": 0.8,
    "joint5l": 0.8,
    "joint6l": 0.8,
    "joint7l": 0.03,
    "joint8l": 0.03,
}
DEFAULT_AMPLITUDE = 0.6          # fallback if joint name not in the dict above
SWEEP_FREQUENCY_HZ = 0.4        # full back-and-forth cycles per second


# ─────────────────────────────────────────────────────────────────────────────
def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    robot = scene["robot"]

    # ── Resolve all joints ───────────────────────────────────────────────────
    all_joints_cfg = SceneEntityCfg("robot", joint_names=[".*"], body_names=[".*"])
    all_joints_cfg.resolve(scene)

    all_joint_names  = robot.data.joint_names          # list[str]  length = N
    all_body_names   = robot.data.body_names           # list[str]  length = M
    num_joints       = len(all_joint_names)
    num_bodies       = len(all_body_names)

    print("\n" + "=" * 60)
    print(f"  Found {num_joints} joints and {num_bodies} links (bodies)")
    print("=" * 60)
    for i, n in enumerate(all_joint_names):
        print(f"  joint [{i:02d}]  {n}")
    print("-" * 60)
    for i, n in enumerate(all_body_names):
        print(f"  body  [{i:02d}]  {n}")
    print("=" * 60 + "\n")

    # ── Visualisation markers (one axis-frame per body, shown in link phase) ─
    marker_cfg = FRAME_MARKER_CFG.copy()
    marker_cfg.markers["frame"].scale = (0.08, 0.08, 0.08)
    link_markers = VisualizationMarkers(
        marker_cfg.replace(prim_path="/Visuals/LinkMarkers")
    )

    # ── Inspector state machine ───────────────────────────────────────────────
    # phase = "joints"  →  we sweep joint[current_idx] sinusoidally
    # phase = "links"   →  we freeze all joints, highlight body[current_idx]
    state = {
        "phase":       "joints",
        "current_idx": 0,
        "advance":     False,   # set True by the P-key callback
    }

    # ── P-key callback ────────────────────────────────────────────────────────
    def _on_press_p():
        state["advance"] = True

    # Minimal UI window with a "Next (P)" button (pressing P on keyboard also works)
    ctrl_window = ui.Window("Inspector Controls", width=260, height=110)
    with ctrl_window.frame:
        with ui.VStack(spacing=8):
            ui.Label("Press  P  (keyboard) or click below", alignment=ui.Alignment.CENTER)
            ui.Button("▶  Next joint / link  (P)", clicked_fn=_on_press_p, height=44)

    # Keyboard hook for  P
    try:
        from isaaclab.devices import Se3Keyboard  # use its internal carb hook
        # We only want the P key — wrap with a tiny carb input listener instead
        import carb.input as carb_input
        import omni.appwindow

        _app_window   = omni.appwindow.get_default_app_window()
        _keyboard     = _app_window.get_keyboard()
        _input_iface  = carb_input.acquire_input_interface()

        def _keyboard_cb(event, *args, **kwargs):
            if (event.type == carb_input.KeyboardEventType.KEY_PRESS and
                    event.input == carb_input.KeyboardInput.P):
                _on_press_p()
            return True

        _sub = _input_iface.subscribe_to_keyboard_events(_keyboard, _keyboard_cb)
        print("[INFO] Keyboard listener attached — press  P  in the viewport to advance.")
    except Exception as exc:
        print(f"[WARN] Could not attach keyboard listener: {exc}")
        print("[INFO] Use the UI button to advance instead.")

    # ── Helpers ───────────────────────────────────────────────────────────────
    sim_dt  = sim.get_physics_dt()
    elapsed = 0.0            # seconds since phase/joint began

    def _get_amplitude(joint_name: str) -> float:
        for key, amp in JOINT_AMPLITUDES.items():
            if key.lower() in joint_name.lower():
                return amp
        return DEFAULT_AMPLITUDE

    def _print_current():
        if state["phase"] == "joints":
            idx = state["current_idx"]
            amp = _get_amplitude(all_joint_names[idx])
            print(f"\n[INSPECTOR] ── JOINT PHASE  [{idx+1}/{num_joints}] ──")
            print(f"             Sweeping joint  '{all_joint_names[idx]}'")
            print(f"             Amplitude : ±{amp:.3f} rad   Freq: {SWEEP_FREQUENCY_HZ} Hz")
            print(f"             Press  P  to move to the next joint.\n")
        else:
            idx = state["current_idx"]
            print(f"\n[INSPECTOR] ── LINK PHASE  [{idx+1}/{num_bodies}] ──")
            print(f"             Highlighting body  '{all_body_names[idx]}'")
            print(f"             (axis-frame marker shown at link origin)")
            print(f"             Press  P  to move to the next link.\n")

    _print_current()

    # ── Main loop ─────────────────────────────────────────────────────────────
    while simulation_app.is_running():

        # ── Handle advance request ─────────────────────────────────────────
        if state["advance"]:
            state["advance"] = False
            elapsed          = 0.0

            if state["phase"] == "joints":
                # Reset arm to default before moving to next
                reset_pos = robot.data.default_joint_pos.clone()
                reset_vel = robot.data.default_joint_vel.clone()
                robot.write_joint_state_to_sim(reset_pos, reset_vel)

                state["current_idx"] += 1
                if state["current_idx"] >= num_joints:
                    # All joints done → switch to link phase
                    state["phase"]       = "links"
                    state["current_idx"] = 0
                    print("\n[INSPECTOR] ══ All joints inspected!  Starting LINK phase. ══\n")
            else:
                state["current_idx"] += 1
                if state["current_idx"] >= num_bodies:
                    print("\n[INSPECTOR] ══ All links inspected!  Closing simulation. ══\n")
                    break

            _print_current()

        # ── JOINT PHASE: sinusoidal sweep of the current joint ─────────────
        if state["phase"] == "joints":
            joint_idx  = state["current_idx"]
            joint_name = all_joint_names[joint_idx]
            amplitude  = _get_amplitude(joint_name)

            # Build target = all zeros (default) except the active joint
            target = robot.data.default_joint_pos.clone()   # shape [1, N]
            angle  = amplitude * math.sin(2.0 * math.pi * SWEEP_FREQUENCY_HZ * elapsed)
            target[0, joint_idx] = angle

            robot.set_joint_position_target(target)

        # ── LINK PHASE: freeze joints, show marker at the selected body ────
        else:
            # Keep joints at default (already reset when we entered this phase)
            target = robot.data.default_joint_pos.clone()
            robot.set_joint_position_target(target)

            # Build marker positions: dim all to near-zero, highlight current
            body_idx    = state["current_idx"]
            body_states = robot.data.body_state_w   # [1, M, 13]

            positions  = body_states[0, :, 0:3].clone()   # [M, 3]
            quats      = body_states[0, :, 3:7].clone()   # [M, 4]

            # Repeat shapes to feed into visualizer (needs [N,3] and [N,4])
            link_markers.visualize(positions, quats)

            # Print the current link's world position once per 120 steps (~1.2 s)
            if int(elapsed / sim_dt) % 120 == 0:
                pos = body_states[0, body_idx, 0:3].cpu().numpy()
                print(f"[LINK pos]  '{all_body_names[body_idx]}'  →  "
                      f"x={pos[0]:.4f}  y={pos[1]:.4f}  z={pos[2]:.4f}")

        # ── Physics step ──────────────────────────────────────────────────
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)
        elapsed += sim_dt


# ─────────────────────────────────────────────────────────────────────────────
def main():
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim     = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])

    scene_cfg = InspectorSceneCfg(num_envs=1, env_spacing=2.0)
    scene     = InteractiveScene(scene_cfg)

    sim.reset()
    print("[INFO]: Setup complete — starting Joint Inspector...")

    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
