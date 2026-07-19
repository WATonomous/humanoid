#!/usr/bin/env python3
"""Real-time mjlab (mujoco + mjviser) visualization of the physical bimanual test stand.

Subscribes to /interfacing/motorFeedback (published by the `can` package off the
real CAN bus, see autonomy/interfacing/can/README.md) and drives the matching
joints of urdf/bimanual_arm_curobo.urdf live in a browser-based mjviser scene.
This is read-only: it never publishes MotorCmd, so it cannot move the real arm.

Run inside the `mjlabs` container (see mjlabs_setup.md) while the `interfacing`
container is up and the physical arm is powered:

    ./watod -t mjlabs
    python3 autonomy/simulation/Humanoid_Wato/wato_bimanual_arm/live_arm_mjviser.py

Then open http://<host>:8080 (mjviser's default port, already published by the
mjlabs compose service) to watch qpos update as the real motors move.

Only hardware_mapping.yaml's "left" side is wired/calibrated today, so only that
chain is driven live; the other arm stays in its rest pose. In the URDF, the
robot's left arm is the "L"-suffixed chain (joint1L, joint2l, ..., joint6l) --
confirmed by mirrored origins (y=-0.16355 vs y=+0.16355 relative to base_link).
"Left" here means the robot's own left arm, i.e. it appears on your RIGHT if
you're standing facing the test stand.

The gripper (hardware_mapping.yaml can_id 0x15, command-frame units 0-100) has no
confirmed scale to the prismatic finger joints' travel in meters (joint7l/joint8l),
so it's intentionally left undriven -- see LABEL_TO_URDF_JOINT.
"""
import argparse
import math
import threading
import time
from pathlib import Path

import mujoco
import rclpy
import viser
import yaml
from mjviser import ViserMujocoScene
from rclpy.node import Node

from common_msgs.msg import MotorFeedback

_SCRIPT_DIR = Path(__file__).resolve().parent
_URDF_PATH = _SCRIPT_DIR / "urdf" / "bimanual_arm_curobo.urdf"
_DEFAULT_MAPPING = (
    _SCRIPT_DIR.parent.parent.parent
    / "behaviour" / "joint_command" / "config" / "hardware_mapping.yaml"
)

# hardware_mapping.yaml label -> bimanual_arm_curobo.urdf joint name, per side.
# Both chains are a serial shoulder(x3)-elbow(x2)-wrist(x1)-gripper(x2) stack in the
# same joint order as hardware_mapping.yaml / ros_bridge.py's JOINT_INDEX, matching
# how left_arm_assembly.urdf's descriptively-named joints line up 1:1 in that script.
#
# The "L"-suffixed chain (joint1L, joint2l, ...) IS the robot's left arm: confirmed by
# ../../Teleop/keyboard_based_teleoperation/bimanual_arm_cfg.py's LEFT_ARM_JOINTS list
# and its "left_shoulder"/"left_elbow"/"left_wrist" ImplicitActuatorCfg groups, sourced
# from Isaac Sim Physics Inspector. (A previous version of this file had this swapped --
# the likely real cause of that was the now-removed clamp bug above making the correctly-
# mapped chain look frozen, not an actual left/right mixup.)
LABEL_TO_URDF_JOINT = {
    "left": {
        "shoulder_pitch": "joint1L",
        "shoulder_roll": "joint2l",
        "shoulder_yaw": "joint3l",
        "elbow_pitch": "joint4l",
        "elbow_roll": "joint5l",
        "wrist_pitch": "joint6l",
    },
    "right": {
        "shoulder_pitch": "joint1",
        "shoulder_roll": "joint2",
        "shoulder_yaw": "joint3",
        "elbow_pitch": "joint4",
        "elbow_roll": "joint5",
        "wrist_pitch": "joint6",
    },
}


def load_can_id_map(
    mapping_path: Path,
    hw_side: str,
    urdf_side: str,
    flip_labels: set[str] = frozenset(),
    offset_labels: dict[str, float] | None = None,
) -> dict[int, dict]:
    """hardware_mapping.yaml -> {can_id: {label, urdf_joint, direction, zero_offset, limits}}.

    hw_side picks which hardware_mapping.yaml section (real motors / CAN ids / calibration)
    to read; urdf_side picks which URDF arm those angles drive. They differ when a physical
    arm is mounted/wired as the opposite hand and you want the same feedback projected onto
    the other arm in the scene (e.g. hw_side="left", urdf_side="right").
    """
    with open(mapping_path) as f:
        config = yaml.safe_load(f)[hw_side]

    label_to_joint = LABEL_TO_URDF_JOINT[urdf_side]
    can_id_map = {}
    for group, joints in config.items():
        for name, cfg in joints.items():
            label = f"{group}_{name}"
            urdf_joint = label_to_joint.get(label)
            if urdf_joint is None:
                continue
            # Extra sign flip on top of hardware_mapping's own `direction`, applied only to
            # this display projection (never to real MotorCmd). Needed per-joint when
            # projecting onto the mirror-image opposite arm so it moves the intuitive way.
            direction = int(cfg["direction"]) * (-1 if label in flip_labels else 1)
            can_id_map[int(cfg["can_id"])] = {
                "label": label,
                "urdf_joint": urdf_joint,
                "direction": direction,
                "zero_offset": float(cfg["zero_offset"]),
                # Viewer-only constant added to the displayed angle (degrees). Use when the
                # URDF's zero for a joint sits a fixed amount off the motor's zeroed frame.
                # Never fed back into any MotorCmd.
                "display_offset": float((offset_labels or {}).get(label, 0.0)),
                "lower_limit": float(cfg["lower_limit"]),
                "upper_limit": float(cfg["upper_limit"]),
            }
    return can_id_map


class FeedbackListener(Node):
    """Keeps the latest command-frame joint angle (deg) per URDF joint, from real CAN feedback."""

    def __init__(self, can_id_map: dict[int, dict]):
        super().__init__("real_arm_mjviser_bridge")
        self.can_id_map = can_id_map
        self.lock = threading.Lock()
        self.latest_deg: dict[str, float] = {}
        self.create_subscription(
            MotorFeedback, "/interfacing/motorFeedback", self._on_feedback, 20
        )

    def _on_feedback(self, msg: MotorFeedback) -> None:
        cfg = self.can_id_map.get(int(msg.motor_id))
        if cfg is None:
            return
        # Invert the calibration transform ros_bridge.py applies to outgoing MotorCmd:
        # motor_deg = direction * (cmd_deg - zero_offset)  =>  cmd_deg = zero_offset + direction * motor_deg
        #
        # Deliberately NOT clamped to hardware_mapping.yaml's lower/upper_limit here --
        # those exist to bound outgoing MotorCmd for safety, but this script only reads
        # feedback and never commands anything. Clamping display values froze any joint
        # whose real range exceeds its (still-uncalibrated, zero_offset=0) placeholder
        # limit, making it look motionless in sim while it kept moving for real.
        joint_deg = (
            cfg["zero_offset"] + cfg["direction"] * float(msg.position) + cfg["display_offset"]
        )
        with self.lock:
            self.latest_deg[cfg["urdf_joint"]] = joint_deg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-side", default="left", choices=["left", "right"],
                        help="hardware side: which hardware_mapping.yaml section / real motors to read")
    parser.add_argument("--urdf-side", default=None, choices=["left", "right"],
                        help="which URDF arm to drive with that feedback (default: same as --arm-side). "
                             "Set to the opposite of --arm-side to project onto the other hand.")
    parser.add_argument("--flip", nargs="*", default=[], metavar="LABEL",
                        help="hardware_mapping labels whose sign to invert for this projection "
                             "(e.g. shoulder_roll) -- useful when projecting onto the mirror-image "
                             "opposite arm and a joint moves the wrong way.")
    parser.add_argument("--offset", nargs="*", default=[], metavar="LABEL=DEG",
                        help="viewer-only constant added to a joint's displayed angle, in degrees "
                             "(e.g. shoulder_yaw=90) -- use when the URDF zero for that joint is a "
                             "fixed amount off the motor's zeroed frame. Never touches real commands.")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--hz", type=float, default=30.0, help="scene refresh rate")
    parser.add_argument("--mapping", type=Path, default=_DEFAULT_MAPPING)
    parser.add_argument("--urdf", type=Path, default=_URDF_PATH)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.urdf))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    urdf_side = args.urdf_side or args.arm_side
    offset_labels = {}
    for item in args.offset:
        label, _, val = item.partition("=")
        offset_labels[label.strip()] = float(val)
    can_id_map = load_can_id_map(
        args.mapping, args.arm_side, urdf_side, set(args.flip), offset_labels
    )
    if not can_id_map:
        raise RuntimeError(f"No joints resolved for arm_side={args.arm_side!r} in {args.mapping}")

    qpos_adr = {}
    for cfg in can_id_map.values():
        jid = model.joint(cfg["urdf_joint"]).id
        qpos_adr[cfg["urdf_joint"]] = int(model.jnt_qposadr[jid])

    print(f"Tracking {len(can_id_map)} motors from {args.mapping} "
          f"(hardware={args.arm_side} arm -> driving URDF {urdf_side} arm):")
    for can_id, cfg in sorted(can_id_map.items()):
        flip = "  (flipped)" if cfg["label"] in set(args.flip) else ""
        off = f"  (offset {cfg['display_offset']:+g}°)" if cfg["display_offset"] else ""
        print(f"  0x{can_id:02X} -> {cfg['label']:<14} -> {cfg['urdf_joint']}{flip}{off}")

    rclpy.init()
    node = FeedbackListener(can_id_map)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    server = viser.ViserServer(port=args.port)
    scene = ViserMujocoScene(server, model, num_envs=1)
    scene.update_from_mjdata(data)
    print(f"mjviser live -- open http://localhost:{args.port} to watch the real arm.")

    period = 1.0 / args.hz
    try:
        while rclpy.ok():
            with node.lock:
                latest = dict(node.latest_deg)
            for urdf_joint, deg in latest.items():
                data.qpos[qpos_adr[urdf_joint]] = math.radians(deg)
            mujoco.mj_forward(model, data)
            scene.update_from_mjdata(data)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        server.stop()


if __name__ == "__main__":
    main()
