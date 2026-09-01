"""Export shared badminton assets to autonomy/simulation/badminton/assets/.

Two assets, both derived from scene/params.yaml + the v2 arm URDF (single
sources of truth; regenerate with this script rather than editing outputs):

- wato_arm_v2_racket.urdf: the dual-arm-on-stand URDF with the right-arm
  joint ranges/efforts/damping assigned from params.yaml (the source export
  has every limit as 0/0/0/0), the gripper fingers fixed closed on the
  racket handle, and the racket (visual mesh + collision primitives +
  compiled inertial) attached to link6 by a fixed joint. Arm meshes are
  referenced relative to the output dir; the racket mesh is copied in.
- court.xml: standalone MJCF of the court (floor, lines, net band, posts,
  tape) with the calibrated floor/net solref values.

Usage: uv run python scripts/export_assets.py
"""

import os
import re
import shutil
import sys

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import aero
from build_scene import FINGER_PAD, HANDLE_R, RACKET_R

OUT_DIR = os.path.normpath(os.path.join(ROOT, "..", "assets"))
ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def racket_inertial(p: dict) -> str:
    """Compile the scene's racket geoms (handle capsule + face box) in
    isolation and read back the combined body inertial for the URDF."""
    fo = p["racket"]["face_offset_y"]
    fs = p["racket"]["face_size"]
    handle_lo = fo - 0.525
    spec = mujoco.MjSpec()
    b = spec.worldbody.add_body(name="racket")
    b.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[HANDLE_R, 0, 0],
               fromto=[0, handle_lo, 0, 0, handle_lo + 0.13, 0],
               mass=p["racket"]["handle_mass"])
    b.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX,
               size=[fs[0] / 2, fs[1] / 2, fs[2] / 2], pos=[0, fo, 0],
               mass=p["racket"]["face_mass"])
    m = spec.compile()
    bid = m.body("racket").id
    com = m.body_ipos[bid]
    rpy = Rotation.from_quat(m.body_iquat[bid], scalar_first=True).as_euler("xyz")
    ixx, iyy, izz = m.body_inertia[bid]
    return f"""    <inertial>
      <origin xyz="{com[0]:.6g} {com[1]:.6g} {com[2]:.6g}" rpy="{rpy[0]:.6g} {rpy[1]:.6g} {rpy[2]:.6g}" />
      <mass value="{m.body_mass[bid]:.6g}" />
      <inertia ixx="{ixx:.6g}" ixy="0" ixz="0" iyy="{iyy:.6g}" iyz="0" izz="{izz:.6g}" />
    </inertial>"""


def racket_xml(p: dict) -> str:
    fo = p["racket"]["face_offset_y"]
    fs = p["racket"]["face_size"]
    handle_lo = fo - 0.525
    gp = p["racket"]["grip_pos"]
    rpy = Rotation.from_matrix(RACKET_R).as_euler("xyz")
    return f"""    <!-- racket, held rigidly by the closed gripper (exported by
         stationary/scripts/export_assets.py; values from params.yaml).
         Racket frame: +y = shaft (grip to face), +z = face normal. -->
    <link name="racket">
{racket_inertial(p)}
        <visual>
            <origin xyz="0 {fo} 0" rpy="0 0 0" />
            <geometry>
                <mesh filename="meshes/racket_vis.obj" scale="1 1 1" />
            </geometry>
            <material name="racket_orange">
                <color rgba="0.85 0.30 0.10 1" />
            </material>
        </visual>
        <collision>
            <origin xyz="0 {handle_lo + 0.065:.6g} 0" rpy="-1.5707963 0 0" />
            <geometry>
                <cylinder radius="{HANDLE_R}" length="0.13" />
            </geometry>
        </collision>
        <collision>
            <origin xyz="0 {fo} 0" rpy="0 0 0" />
            <geometry>
                <box size="{fs[0]} {fs[1]} {fs[2]}" />
            </geometry>
        </collision>
    </link>
    <joint name="racket_weld" type="fixed">
        <origin xyz="{gp[0]} {gp[1]} {gp[2]}" rpy="{rpy[0]:.7g} {rpy[1]:.7g} {rpy[2]:.7g}" />
        <parent link="link6" />
        <child link="racket" />
    </joint>
"""


def export_urdf(p: dict) -> str:
    src = open(os.path.normpath(os.path.join(ROOT, p["arm"]["urdf"]))).read()
    src = re.sub(r"<gazebo.*?</gazebo>", "", src, flags=re.S)
    src = re.sub(r"<transmission.*?</transmission>", "", src, flags=re.S)
    src = re.sub(r'(<robot[^>]*>)',
                 r'\1\n<mujoco><compiler balanceinertia="true" '
                 r'discardvisual="false"/></mujoco>', src, count=1)
    src = src.replace("package://armWithStand/meshes/",
                      "../../Humanoid_Wato/wato_arm_v2/meshes/")

    # right-arm joints: ranges/efforts/damping from params (source has 0/0/0/0)
    for i, name in enumerate(ARM_JOINTS):
        lo, hi = p["arm"]["joint_range"][i]
        tau = p["arm"]["torque_limits"][i]
        damp = p["arm"]["joint_damping"][i]
        pat = re.compile(
            rf'(<joint\s+name="{name}"\s+type="revolute">.*?)<limit[^/]*/>',
            re.S)
        repl = (rf'\g<1><limit lower="{lo}" upper="{hi}" effort="{tau}" '
                rf'velocity="0" />\n        <dynamics damping="{damp}" />')
        src, n = pat.subn(repl, src, count=1)
        assert n == 1, f"limit tag not found for {name}"

    # gripper fingers: fix closed on the racket handle (same pose as the sim)
    grip_y = p["racket"]["grip_pos"][1]
    for name, sign in (("joint7", -1.0), ("joint8", +1.0)):
        y = grip_y + sign * (HANDLE_R + FINGER_PAD + 0.001)
        pat = re.compile(rf'<joint\s+name="{name}"\s+type="prismatic">(.*?)</joint>',
                         re.S)
        mm = pat.search(src)
        assert mm, f"{name} not found"
        body = mm.group(1)
        org = re.search(r'<origin\s+xyz="(\S+) (\S+) (\S+)"', body)
        x0, _, z0 = org.groups()
        parent = re.search(r'<parent\s+link="(\w+)"', body).group(1)
        child = re.search(r'<child\s+link="(\w+)"', body).group(1)
        fixed = (f'<joint name="{name}" type="fixed">\n'
                 f'        <origin xyz="{x0} {y:.6g} {z0}" rpy="0 0 0" />\n'
                 f'        <parent link="{parent}" />\n'
                 f'        <child link="{child}" />\n    </joint>')
        src = pat.sub(fixed, src, count=1)

    src = src.replace("</robot>", racket_xml(p) + "</robot>")
    return src


def court_xml(p: dict) -> str:
    court_l = p["court"]["length"]
    court_w = p["court"]["width"]
    net = p["net"]
    net_cz = (net["height_top"] + net["height_bottom"]) / 2.0
    net_hz = (net["height_top"] - net["height_bottom"]) / 2.0
    fs = p["contact"]["floor_solref"]
    ns = p["contact"]["net_solref"]
    return f"""<!-- Standalone badminton court. Generated by
     stationary/scripts/export_assets.py from stationary/scene/params.yaml;
     regenerate rather than editing. Net line at y = 0, +z up. Floor and net
     band collide by default (the stationary env instead disables these and
     declares explicit contact pairs). -->
<mujoco model="badminton_court">
  <asset>
    <texture name="court_grid" type="2d" builtin="checker" rgb1="0.18 0.30 0.22"
             rgb2="0.16 0.28 0.20" width="512" height="512"/>
    <material name="court_mat" texture="court_grid" texrepeat="8 8" reflectance="0.05"/>
    <material name="net_mat" rgba="0.15 0.15 0.15 0.55"/>
  </asset>
  <worldbody>
    <light pos="0 -3 5" dir="0 0.4 -1" diffuse="0.9 0.9 0.9"/>
    <light pos="0 4 5" dir="0 -0.4 -1" diffuse="0.6 0.6 0.6"/>
    <geom name="floor" type="plane" size="{court_w} {court_l} 0.1"
          material="court_mat" solref="{fs[0]} {fs[1]}"/>
    <geom name="line_left" type="box" size="0.02 {court_l / 2} 0.001"
          pos="-{court_w / 2} 0 0.001" rgba="1 1 1 1" contype="0" conaffinity="0"/>
    <geom name="line_right" type="box" size="0.02 {court_l / 2} 0.001"
          pos="{court_w / 2} 0 0.001" rgba="1 1 1 1" contype="0" conaffinity="0"/>
    <geom name="line_far" type="box" size="{court_w / 2} 0.02 0.001"
          pos="0 {court_l / 2} 0.001" rgba="1 1 1 1" contype="0" conaffinity="0"/>
    <geom name="line_near" type="box" size="{court_w / 2} 0.02 0.001"
          pos="0 -{court_l / 2} 0.001" rgba="1 1 1 1" contype="0" conaffinity="0"/>
    <body name="net_body" pos="0 0 0">
      <geom name="net" type="box" size="{court_w / 2} {net["thickness"] / 2} {net_hz}"
            pos="0 0 {net_cz}" material="net_mat" solref="{ns[0]} {ns[1]}"/>
      <geom name="net_post_l" type="cylinder" size="0.02 {net["height_top"] / 2}"
            pos="-{court_w / 2} 0 {net["height_top"] / 2}" rgba="0.3 0.3 0.3 1"
            contype="0" conaffinity="0"/>
      <geom name="net_post_r" type="cylinder" size="0.02 {net["height_top"] / 2}"
            pos="{court_w / 2} 0 {net["height_top"] / 2}" rgba="0.3 0.3 0.3 1"
            contype="0" conaffinity="0"/>
      <geom name="net_tape" type="box" size="{court_w / 2} 0.012 0.02"
            pos="0 0 {net["height_top"] - 0.02}" rgba="1 1 1 1"
            contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
"""


README = """# shared badminton assets

Generated by `stationary/scripts/export_assets.py` from
`stationary/scene/params.yaml` and the v2 arm URDF — regenerate with
`cd stationary && uv run python scripts/export_assets.py` instead of editing.

## wato_arm_v2_racket.urdf

`Humanoid_Wato/wato_arm_v2/urdf/armWithStand.urdf` (dual arm on stand) with:

- right-arm joint1-6 limits assigned: human-like ranges (the source export
  has every limit as 0/0/0/0; there are no measured mechanical stops), effort
  = motor datasheet peak torques (AK10-9 53 Nm shoulders, AK80-9 22 Nm elbow
  group, GL40 II 0.73 Nm wrist), damping = the sim's motor-scaled stability
  values. Velocity limits are left 0 (unspecified, as in the source).
- gripper fingers (joint7/joint8) fixed closed on the racket handle.
- the racket attached to link6 by a fixed joint: visual mesh
  (`meshes/racket_vis.obj`, copied from the stationary build), collision
  cylinder (handle) + box (string bed face), inertial compiled from the
  primitives (90 g total). Racket frame: +y = shaft toward the face,
  +z = face normal (link6 -z and +x respectively at qpos = 0).
- left-arm joints untouched (revolute with 0/0/0/0 limits — no data);
  freeze or assign ranges before actuating them.
- arm meshes referenced by relative path
  (`../../Humanoid_Wato/wato_arm_v2/meshes/`); loads directly in MuJoCo.

## court.xml

Standalone MJCF court: floor plane, court outline lines, net band
(collidable, with the calibrated solref), posts and tape. Net line at y = 0,
+z up. Combine with other models via `MjSpec.attach` or copy the worldbody.
"""


def main():
    p = aero.load_params()
    os.makedirs(os.path.join(OUT_DIR, "meshes"), exist_ok=True)
    shutil.copy(os.path.join(ROOT, "scene", "assets", "racket_vis.obj"),
                os.path.join(OUT_DIR, "meshes", "racket_vis.obj"))
    urdf_path = os.path.join(OUT_DIR, "wato_arm_v2_racket.urdf")
    with open(urdf_path, "w") as f:
        f.write(export_urdf(p))
    with open(os.path.join(OUT_DIR, "court.xml"), "w") as f:
        f.write(court_xml(p))
    with open(os.path.join(OUT_DIR, "README.md"), "w") as f:
        f.write(README)

    # validation: both assets must load in mujoco
    m = mujoco.MjModel.from_xml_path(urdf_path)
    names = [m.joint(i).name for i in range(m.njnt)]
    assert all(j in names for j in ARM_JOINTS)
    assert "joint7" not in names and "joint8" not in names
    j1 = m.joint("joint1")
    assert np.allclose(j1.range, p["arm"]["joint_range"][0])
    mc = mujoco.MjModel.from_xml_path(os.path.join(OUT_DIR, "court.xml"))
    assert mc.ngeom == 9  # floor + 4 lines + net band + 2 posts + tape
    print(f"wrote {OUT_DIR}: urdf ({m.njnt} joints, {m.nbody} bodies), "
          f"court ({mc.ngeom} geoms), racket mesh, README")


if __name__ == "__main__":
    main()
