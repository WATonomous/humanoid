"""Build the cuRobo robot config for the wato_bimanual_arm left arm.

Three steps, one reproducible entrypoint:
  1. Write a patched URDF copy (bimanual_arm_curobo.urdf) with the real
     Physics Inspector joint limits — the SolidWorks export has every limit
     as (0, 0), which would clamp all joints to zero in cuRobo.
  2. Run cuRobo's RobotBuilder to fit collision spheres (MorphIt) and the
     self-collision ignore matrix from the URDF meshes.
  3. Post-process the YAML: lock the right arm + both grippers at their
     default pose, set tool frame link6l, default joint position, and an
     `attached_object` link for carrying grasped objects during transit.

Run inside the simulation_il container:
    $PYTHON build_wato_robot_cfg.py [--sphere-density 1.0] [--skip-fit]
"""
import argparse
import os
import xml.etree.ElementTree as ET

import yaml

import wato_constants as wc

_EFFORT = {"revolute": 50.0, "prismatic": 30.0}
_VELOCITY = {"revolute": 6.0, "prismatic": 0.2}


def write_patched_urdf() -> str:
    tree = ET.parse(wc.URDF_PATH)
    root = tree.getroot()
    patched = []
    for joint in root.findall("joint"):
        name = joint.get("name")
        jtype = joint.get("type")
        if name not in wc.JOINT_POS_LIMITS or jtype not in _EFFORT:
            continue
        limit = joint.find("limit")
        if limit is None:
            limit = ET.SubElement(joint, "limit")
        lo, hi = wc.JOINT_POS_LIMITS[name]
        limit.set("lower", repr(lo))
        limit.set("upper", repr(hi))
        limit.set("effort", repr(_EFFORT[jtype]))
        limit.set("velocity", repr(_VELOCITY[jtype]))
        patched.append(name)
    tree.write(wc.CUROBO_URDF_PATH, xml_declaration=True, encoding="utf-8")
    print(f"Patched limits for {len(patched)} joints -> {wc.CUROBO_URDF_PATH}")
    return wc.CUROBO_URDF_PATH


def fit_spheres(urdf_path: str, out_path: str, sphere_density: float, seed: int,
                protrusion_weight: float) -> None:
    import numpy as np
    import torch

    from curobo.robot_builder import RobotBuilder

    np.random.seed(seed)
    torch.manual_seed(seed)

    builder = RobotBuilder(
        urdf_path=urdf_path,
        asset_path=os.path.dirname(urdf_path),
        tool_frames=[wc.RIGHT_EE_BODY],
    )
    builder.fit_collision_spheres(
        sphere_density=sphere_density,
        protrusion_weight=protrusion_weight,
        compute_metrics=True,
    )
    print(f"Fitted {builder.num_spheres} spheres across {len(builder.collision_link_names)} links")
    for link, m in (builder.link_metrics or {}).items():
        print(f"  {link:12s} n={m.num_spheres:3d} cover={m.coverage*100:5.1f}% protr={m.protrusion*100:5.1f}%")
    builder.compute_collision_matrix(num_samples=1000)
    config = builder.build()
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    builder.save(config, out_path)
    print(f"Saved base robot config -> {out_path}")


def post_process(yml_path: str) -> None:
    with open(yml_path) as f:
        cfg = yaml.safe_load(f)
    kin = cfg["kinematics"]

    kin["tool_frames"] = [wc.RIGHT_EE_BODY]
    kin["lock_joints"] = dict(wc.LOCKED_JOINTS)

    cspace = kin["cspace"]
    joint_names = cspace["joint_names"]
    missing = [j for j in wc.JOINT_POS_LIMITS if j not in joint_names]
    if missing:
        raise RuntimeError(f"URDF joints missing from cspace: {missing}")
    cspace["default_joint_position"] = [wc.DEFAULT_JOINT_POS[j] for j in joint_names]

    # Attached-object link for carrying grasped objects (franka.yml pattern).
    extra_links = kin.get("extra_links") or {}
    extra_links["attached_object"] = {
        "parent_link_name": wc.RIGHT_EE_BODY,
        "link_name": "attached_object",
        "joint_name": "attach_joint",
        "joint_type": "FIXED",
        "fixed_transform": [*wc.FINGERTIP_OFFSET_IN_WRIST, 1.0, 0.0, 0.0, 0.0],
    }
    kin["extra_links"] = extra_links
    extra_spheres = kin.get("extra_collision_spheres") or {}
    extra_spheres["attached_object"] = 8
    kin["extra_collision_spheres"] = extra_spheres
    ignore = kin.get("self_collision_ignore") or {}
    for link in (wc.RIGHT_EE_BODY, "link7l", "link8l", "link5l"):
        ignore.setdefault(link, [])
        if "attached_object" not in ignore[link]:
            ignore[link].append("attached_object")
    kin["self_collision_ignore"] = ignore
    buf = kin.get("self_collision_buffer") or {}
    buf["attached_object"] = 0.0
    kin["self_collision_buffer"] = buf
    kin["grasp_contact_link_names"] = ["link7l", "link8l", "attached_object"]

    # The physical robot holds the default pose with PhysX self-collisions
    # enabled, so any sphere pair colliding at the default pose is a fit
    # artifact. Add those pairs to the ignore matrix (with a loud report).
    artifact_pairs = _default_pose_collisions(kin)
    for a, b in artifact_pairs:
        ignore.setdefault(a, [])
        if b not in ignore[a] and a not in ignore.get(b, []):
            ignore[a].append(b)
    if artifact_pairs:
        print(f"NOTE: ignored {len(artifact_pairs)} default-pose artifact pairs: {artifact_pairs}")

    with open(yml_path, "w") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    print(f"Post-processed {yml_path}: lock_joints={list(kin['lock_joints'])},")
    print(f"  tool_frames={kin['tool_frames']}, attached_object under {wc.RIGHT_EE_BODY}")


def _default_pose_collisions(kin_cfg: dict) -> list:
    """Link pairs whose fitted spheres collide at the default joint pose."""
    import tempfile

    import numpy as np
    import torch

    from curobo.kinematics import Kinematics, KinematicsCfg
    from curobo.types import JointState

    with tempfile.NamedTemporaryFile("w", suffix=".yml", delete=False) as f:
        yaml.safe_dump({"kinematics": kin_cfg}, f, sort_keys=False)
        tmp = f.name
    kin = Kinematics(KinematicsCfg.from_robot_yaml_file(tmp))
    names = kin.joint_names
    q0 = torch.tensor([[wc.DEFAULT_JOINT_POS[n] for n in names]], device="cuda")
    st = kin.compute_kinematics(JointState.from_position(q0, joint_names=names))
    sph = st.robot_spheres.view(-1, 4).detach().cpu().numpy()
    os.unlink(tmp)

    # Sphere order follows collision_link_names (loader iteration order),
    # NOT the collision_spheres dict key order.
    labels = []
    for link in kin_cfg["collision_link_names"]:
        if link == "attached_object":
            continue
        labels += [link] * len(kin_cfg["collision_spheres"][link])
    labels += ["attached_object"] * (sph.shape[0] - len(labels))
    ignore = kin_cfg.get("self_collision_ignore") or {}

    pairs = {}
    for i in range(len(sph)):
        for j in range(i + 1, len(sph)):
            a, b = labels[i], labels[j]
            if a == b or "attached_object" in (a, b):
                continue
            if b in ignore.get(a, []) or a in ignore.get(b, []):
                continue
            d = np.linalg.norm(sph[i, :3] - sph[j, :3]) - sph[i, 3] - sph[j, 3]
            if d < 0:
                key = tuple(sorted((a, b)))
                pairs[key] = min(pairs.get(key, 0.0), float(d))
    for key, pen in sorted(pairs.items(), key=lambda kv: kv[1]):
        print(f"  default-pose overlap {key}: {pen * 1000:.1f} mm")
    return sorted(pairs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sphere-density", type=float, default=1.0)
    parser.add_argument("--protrusion-weight", type=float, default=100.0,
                        help="MorphIt protrusion penalty (default 100; curobo default 10 "
                             "over-inflates this robot's thin links).")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--skip-fit", action="store_true",
                        help="Only re-run URDF patch + YAML post-processing.")
    args = parser.parse_args()

    urdf = write_patched_urdf()
    if not args.skip_fit:
        fit_spheres(urdf, wc.CUROBO_ROBOT_YML, args.sphere_density, args.seed,
                    args.protrusion_weight)
    post_process(wc.CUROBO_ROBOT_YML)


if __name__ == "__main__":
    main()
