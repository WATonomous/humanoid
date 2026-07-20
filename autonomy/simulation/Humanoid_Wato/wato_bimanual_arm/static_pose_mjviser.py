#!/usr/bin/env python3
"""Show a fixed, hand-specified joint pose in mjviser -- no CAN/ROS/motors involved.

Temporary visualization helper: loads urdf/bimanual_arm_curobo.urdf and sets qpos to a
static set of joint angles (degrees) passed on the command line, then serves it in the
browser same as live_arm_mjviser.py. With no --joint args, shows the model's raw default
spawn pose (qpos=0 for every joint, i.e. the URDF's own built-in geometric zero).

Usage:
    python3 static_pose_mjviser.py                          # default spawn pose (qpos=0)
    python3 static_pose_mjviser.py --joint joint1L=139.2 joint2l=66.1 ...
"""
import argparse
import math
from pathlib import Path

import mujoco
import viser
from mjviser import ViserMujocoScene

_URDF_PATH = Path(__file__).resolve().parent / "urdf" / "bimanual_arm_curobo.urdf"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint", nargs="*", default=[], metavar="NAME=DEG",
                        help="URDF joint name -> angle in degrees, e.g. joint1L=139.2")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--urdf", type=Path, default=_URDF_PATH)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.urdf))
    data = mujoco.MjData(model)

    for item in args.joint:
        name, _, deg_str = item.partition("=")
        name = name.strip()
        deg = float(deg_str)
        jid = model.joint(name).id
        qpos_adr = int(model.jnt_qposadr[jid])
        data.qpos[qpos_adr] = math.radians(deg)
        print(f"  {name} -> {deg}deg ({math.radians(deg):.4f} rad)")

    if not args.joint:
        print("  (no --joint given: showing raw default spawn pose, qpos=0 for all joints)")

    mujoco.mj_forward(model, data)

    server = viser.ViserServer(port=args.port)
    scene = ViserMujocoScene(server, model, num_envs=1)
    scene.update_from_mjdata(data)
    print(f"Static pose shown -- open http://localhost:{args.port}")
    print("Ctrl+C to stop.")
    try:
        while True:
            import time
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == "__main__":
    main()
