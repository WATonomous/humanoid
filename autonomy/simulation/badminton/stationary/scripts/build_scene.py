"""Build scene/badminton.xml from params.yaml + the existing arm URDF.

The arm (wato_arm_v2/armWithStand.urdf, dual arm on a stand) is a fixed input:
its kinematics and inertials are imported untouched. This script only
- freezes the left arm and both grippers (only right joint1-6 stay actuated),
- assigns joint ranges/efforts (the v2 URDF exports every limit as 0/0/0/0),
- places the stand on the floor behind the net, yawed to face it,
- welds the two-layer racket (visual mesh + collision primitives) into the
  right gripper (link6),
- adds the two-layer shuttle, court, net, explicit contact pairs, actuators.

Conventions: net line at y = 0, arm on the y < 0 side, +z up.
Geom groups: 2 = visual meshes, 3 = collision primitives (sim_lab toggles).
Contacts: explicit <pair> entries only; every geom has contype=conaffinity=0.

Usage: uv run python scripts/build_scene.py   (writes scene/badminton.xml)
"""

import os
import re
import sys

import mujoco
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import aero

SCENE_DIR = os.path.join(ROOT, "scene")
OUT_PATH = os.path.join(SCENE_DIR, "badminton.xml")

ARM_PREFIX = "arm_"
HANDLE_R = 0.0145      # m, racket handle capsule radius
FINGER_PAD = 0.0038    # m, finger-pad surface offset from the finger origin

# racket frame relative to link6 (all link frames are parallel to the base
# frame at qpos=0): racket +y (shaft, grip->face) -> link6 -z (down the
# gripper), racket +z (face normal) -> link6 +x (robot forward).
# Columns are the images of the racket x, y, z axes.
RACKET_R = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def patch_urdf(urdf_path: str, mesh_dir: str, out_path: str) -> None:
    """Strip gazebo/transmission tags and add the mujoco compiler extension."""
    src = open(urdf_path).read()
    src = re.sub(r"<gazebo.*?</gazebo>", "", src, flags=re.S)
    src = re.sub(r"<transmission.*?</transmission>", "", src, flags=re.S)
    ext = (f'<mujoco><compiler meshdir="{mesh_dir}" strippath="true" '
           f'balanceinertia="true" discardvisual="false"/></mujoco>')
    src = re.sub(r"(<robot[^>]*>)", r"\1\n" + ext, src, count=1)
    with open(out_path, "w") as f:
        f.write(src)


def scene_spec_xml(p: dict) -> str:
    court_l = p["court"]["length"]
    court_w = p["court"]["width"]
    net = p["net"]
    dt = p["integrator"]["dt"]
    g = p["gravity"]
    shuttle = p["shuttle"]
    net_cz = (net["height_top"] + net["height_bottom"]) / 2.0
    net_hz = (net["height_top"] - net["height_bottom"]) / 2.0
    lt = p["control"]["landing_target"]
    return f"""
<mujoco model="badminton_receive">
  <option timestep="{dt}" gravity="0 0 -{g}" integrator="{p["integrator"]["method"]}"/>
  <visual>
    <scale contactwidth="0.05" contactheight="0.02" forcewidth="0.03"/>
    <map force="0.5"/>
    <quality shadowsize="4096"/>
  </visual>
  <asset>
    <mesh name="racket_vis" file="assets/racket_vis.obj"/>
    <mesh name="shuttle_vis" file="assets/shuttle_vis.obj"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.18 0.30 0.22"
             rgb2="0.16 0.28 0.20" width="512" height="512"/>
    <material name="court_mat" texture="grid" texrepeat="8 8" reflectance="0.05"/>
    <material name="net_mat" rgba="0.15 0.15 0.15 0.55"/>
    <material name="racket_mat" rgba="0.85 0.30 0.10 1"/>
    <material name="shuttle_mat" rgba="0.95 0.95 0.90 1"/>
  </asset>
  <worldbody>
    <light pos="0 -3 5" dir="0 0.4 -1" diffuse="0.9 0.9 0.9"/>
    <light pos="0 4 5" dir="0 -0.4 -1" diffuse="0.6 0.6 0.6"/>
    <geom name="floor" type="plane" size="{court_w} {court_l} 0.1"
          material="court_mat" contype="1" conaffinity="0"/>
    <!-- court outline, visual only -->
    <geom name="line_left" type="box" group="2"
          size="0.02 {court_l/2} 0.001" pos="-{court_w/2} 0 0.001"
          rgba="1 1 1 1" contype="0" conaffinity="0"/>
    <geom name="line_right" type="box" group="2"
          size="0.02 {court_l/2} 0.001" pos="{court_w/2} 0 0.001"
          rgba="1 1 1 1" contype="0" conaffinity="0"/>
    <geom name="line_far" type="box" group="2"
          size="{court_w/2} 0.02 0.001" pos="0 {court_l/2} 0.001"
          rgba="1 1 1 1" contype="0" conaffinity="0"/>
    <geom name="line_near" type="box" group="2"
          size="{court_w/2} 0.02 0.001" pos="0 -{court_l/2} 0.001"
          rgba="1 1 1 1" contype="0" conaffinity="0"/>
    <site name="landing_target" pos="{lt[0]} {lt[1]} 0.01" size="0.15 0.15 0.002"
          type="ellipsoid" rgba="0.9 0.8 0.1 0.5"/>
    <!-- net: collision band from height_bottom to height_top, shuttle-only -->
    <body name="net_body" pos="0 0 0">
      <geom name="net" type="box" group="3"
            size="{court_w/2} {net["thickness"]/2} {net_hz}"
            pos="0 0 {net_cz}" material="net_mat" contype="0" conaffinity="0"/>
      <geom name="net_post_l" type="cylinder" group="2" size="0.02 {net["height_top"]/2}"
            pos="-{court_w/2} 0 {net["height_top"]/2}" rgba="0.3 0.3 0.3 1"
            contype="0" conaffinity="0"/>
      <geom name="net_post_r" type="cylinder" group="2" size="0.02 {net["height_top"]/2}"
            pos="{court_w/2} 0 {net["height_top"]/2}" rgba="0.3 0.3 0.3 1"
            contype="0" conaffinity="0"/>
      <geom name="net_tape" type="box" group="2"
            size="{court_w/2} 0.012 0.02" pos="0 0 {net["height_top"] - 0.02}"
            rgba="1 1 1 1" contype="0" conaffinity="0"/>
    </body>
    <!-- shuttle: two layers; free joint. Skirt sphere carries the mass and
         the net/floor contacts; the cork sphere (surface at the mesh cork
         tip) carries the racket-face contact so a cork-first arrival hits
         where the mesh shows it hitting. -->
    <body name="shuttle" pos="0 5 0.1">
      <freejoint name="shuttle_free"/>
      <geom name="shuttle_col" type="sphere" group="3" size="{shuttle["radius"]}"
            mass="{shuttle["mass"]}" rgba="0.9 0.9 0.9 0.3"
            contype="0" conaffinity="0"/>
      <geom name="shuttle_cork" type="sphere" group="3"
            size="{shuttle["cork_radius"]}" pos="0 0 {shuttle["cork_center_z"]}"
            mass="0" rgba="0.9 0.7 0.7 0.3" contype="0" conaffinity="0"/>
      <geom name="shuttle_mesh" type="mesh" mesh="shuttle_vis" group="2"
            material="shuttle_mat" mass="0" contype="0" conaffinity="0"/>
      <site name="shuttle_site" size="0.005"/>
    </body>
  </worldbody>
</mujoco>
"""


def main():
    p = aero.load_params()

    arm_joints = p["arm"]["joints"]
    urdf_path = os.path.normpath(os.path.join(ROOT, p["arm"]["urdf"]))
    mesh_dir = os.path.normpath(
        os.path.join(os.path.dirname(os.path.dirname(urdf_path)), "meshes"))
    patched = os.path.join(SCENE_DIR, "assets", "armWithStand_patched.urdf")
    patch_urdf(urdf_path, mesh_dir, patched)

    arm = mujoco.MjSpec.from_file(patched)

    # visual stopgap (the end effector is expected to be replaced): squeeze
    # the frozen right-gripper fingers onto the racket handle, 1 mm shy of
    # touching so the workspace self-collision check sees no contact
    grip_y = p["racket"]["grip_pos"][1]
    for fname, sign in (("link7", -1.0), ("link8", +1.0)):
        b = arm.body(fname)
        pos = np.array(b.pos)
        pos[1] = grip_y + sign * (HANDLE_R + FINGER_PAD + 0.001)
        b.pos = pos

    # freeze everything but the right arm: delete the left-arm and gripper
    # joints (their bodies stay, rigid at the zero pose)
    for j in list(arm.joints):
        if j.name not in arm_joints:
            arm.delete(j)
        else:
            # 3-vector in mujoco 3.11 (per-DOF); hinge uses the first entry
            j.damping = np.full(
                3, p["arm"]["joint_damping"][arm_joints.index(j.name)])
            j.armature = p["arm"]["joint_armature"]
            # the v2 URDF exports limits as 0/0/0/0; assign real ones
            j.limited = mujoco.mjtLimited.mjLIMITED_TRUE
            j.range = p["arm"]["joint_range"][arm_joints.index(j.name)]
            tau = p["arm"]["torque_limits"][arm_joints.index(j.name)]
            j.actfrclimited = mujoco.mjtLimited.mjLIMITED_TRUE
            j.actfrcrange = [-tau, tau]

    # visual mesh copies -> group 2; collision copies -> group 3.
    # Collision geoms get contype=1, conaffinity=0: with conaffinity 0
    # everywhere no contype-based pair ever matches (contacts are exclusively
    # the explicit <pair> entries), but the compiler still builds mesh convex
    # hulls, which the workspace self-collision check enables at runtime.
    for gm in arm.geoms:
        if gm.group == 1:
            gm.group = 2
            gm.contype = 0
            gm.conaffinity = 0
        else:
            gm.group = 3
            gm.contype = 1
            gm.conaffinity = 0

    # absolute mesh paths during the build; rewritten to scene-relative at save
    for mesh in arm.meshes:
        mesh.file = os.path.join(mesh_dir, os.path.basename(mesh.file))
    arm.meshdir = ""

    scene = mujoco.MjSpec.from_string(scene_spec_xml(p))
    scene.meshdir = SCENE_DIR

    yaw = np.deg2rad(p["arm"]["mount_yaw_deg"])
    frame = scene.worldbody.add_frame(
        pos=[p["arm"].get("base_x", 0.0), p["arm"]["base_y"],
             p["arm"]["mount_height"]],
        quat=[np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
    scene.attach(arm, prefix=ARM_PREFIX, frame=frame)

    # weld the racket into the right gripper
    racket_quat = np.zeros(4)
    mujoco.mju_mat2Quat(racket_quat, RACKET_R.flatten())
    palm = scene.body(ARM_PREFIX + p["arm"]["palm_body"])
    racket = palm.add_body(name="racket", pos=p["racket"]["grip_pos"],
                           quat=racket_quat)
    fo = p["racket"]["face_offset_y"]
    fs = p["racket"]["face_size"]
    handle_lo = fo - 0.525          # butt of a 0.665 m racket, head half 0.14
    racket.add_geom(
        name="racket_handle", type=mujoco.mjtGeom.mjGEOM_CAPSULE, group=3,
        size=[HANDLE_R, 0, 0], fromto=[0, handle_lo, 0, 0, handle_lo + 0.13, 0],
        mass=p["racket"]["handle_mass"], contype=1, conaffinity=0,
        rgba=[0.2, 0.2, 0.2, 1])
    racket.add_geom(
        name="racket_face", type=mujoco.mjtGeom.mjGEOM_BOX, group=3,
        size=[fs[0] / 2, fs[1] / 2, fs[2] / 2], pos=[0, fo, 0],
        mass=p["racket"]["face_mass"], contype=1, conaffinity=0,
        rgba=[0.9 , 0.5, 0.2, 0.35])
    racket.add_geom(
        name="racket_mesh", type=mujoco.mjtGeom.mjGEOM_MESH, meshname="racket_vis",
        group=2, pos=[0, fo, 0], quat=[1, 0, 0, 0], material="racket_mat",
        mass=0, contype=0, conaffinity=0)
    racket.add_site(name="face_center", pos=[0, fo, 0], size=[0.008, 0.008, 0.008],
                    rgba=[0.1, 0.9, 0.1, 0.8])

    # explicit contact pairs: the only collisions in the model
    c = p["contact"]
    pair = scene.add_pair()
    pair.geomname1, pair.geomname2 = "shuttle_cork", "racket_face"
    pair.solref = np.array(c["face_solref"])
    pair.solimp = np.array(c["face_solimp"] + [0.5, 2.0])  # last two: defaults
    pair = scene.add_pair()
    pair.geomname1, pair.geomname2 = "shuttle_col", "floor"
    pair.solref = np.array(c["floor_solref"])
    pair = scene.add_pair()
    pair.geomname1, pair.geomname2 = "shuttle_cork", "floor"
    pair.solref = np.array(c["floor_solref"])
    pair = scene.add_pair()
    pair.geomname1, pair.geomname2 = "shuttle_col", "net"
    pair.solref = np.array(c["net_solref"])

    # position actuators on the six arm joints
    for jname, kp, kv in zip(arm_joints, p["control"]["kp"], p["control"]["kv"]):
        act = scene.add_actuator()
        act.name = jname
        act.target = ARM_PREFIX + jname
        act.trntype = mujoco.mjtTrn.mjTRN_JOINT
        act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        act.gainprm[0] = kp
        act.biasprm[1] = -kp
        act.biasprm[2] = -kv
        act.ctrlrange = [-3.1415, 3.1415]
    # home keyframe: arm zeros, shuttle parked on the far court
    model = scene.compile()
    key = scene.add_key()
    key.name = "home"
    qpos = np.zeros(model.nq)
    jid = model.joint("shuttle_free").id
    adr = model.jnt_qposadr[jid]
    qpos[adr:adr + 7] = [0, 5.0, 0.1, 1, 0, 0, 0]
    # guard-like ready pose: shoulder pitched forward, wrist turned so the
    # face is at guard height facing the net (episodes overwrite this via IK)
    for jname, ang in [(arm_joints[0], -np.pi / 2), (arm_joints[4], np.pi / 2),
                       (arm_joints[5], -np.pi / 2)]:
        jid = model.joint(ARM_PREFIX + jname).id
        qpos[model.jnt_qposadr[jid]] = ang
    key.qpos = qpos

    scene.compile()

    # serialize with absolute mesh paths, then rewrite them scene-relative in
    # the XML text so the saved file is portable within the repo
    xml = scene.to_xml()
    rel_mesh_dir = os.path.relpath(mesh_dir, SCENE_DIR)
    xml = xml.replace(mesh_dir + os.sep, rel_mesh_dir + "/")
    xml = xml.replace(SCENE_DIR + os.sep, "")
    with open(OUT_PATH, "w") as f:
        f.write(xml)

    # validate the saved file end-to-end
    model = mujoco.MjModel.from_xml_path(OUT_PATH)
    print(f"wrote {OUT_PATH}")
    print(f"nq={model.nq} nv={model.nv} nu={model.nu} ngeom={model.ngeom} "
          f"npair={model.npair} nmesh={model.nmesh}")
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    sid = model.site("face_center").id
    print("face_center at home:", data.site_xpos[sid].round(3))
    print("racket subtree mass:",
          round(float(model.body_subtreemass[model.body("racket").id]), 3))


if __name__ == "__main__":
    main()
