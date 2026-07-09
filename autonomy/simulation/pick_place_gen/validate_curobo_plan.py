"""M1 validation: cuRobo planning for the wato_bimanual_arm left arm.

Runs WITHOUT Isaac Sim (pure cuRobo). Two stages:

  --fk-only     Print calibration: default wrist/fingertip pose, joint order.
  (default)     Plan a grid of top-down fingertip goals over a workspace box
                above a table slab and report plan success rate + timing.

Run inside the simulation_il container:
    $PYTHON validate_curobo_plan.py --fk-only
    $PYTHON validate_curobo_plan.py --box-center X Y Z --box-extent DX DY DZ
"""
import argparse
import math
import time

import numpy as np
import torch

import wato_constants as wc


def top_down_wrist_quat(yaw: float) -> np.ndarray:
    """World-from-wrist quaternion (wxyz) with the approach axis (wrist -Y)
    pointing down (world -Z) and wrist X horizontal at `yaw`."""
    c, s = math.cos(yaw), math.sin(yaw)
    # Columns: wrist X -> (c, s, 0), wrist Y -> (0, 0, 1), wrist Z -> (s, -c, 0)
    m = np.array([[c, 0.0, s], [s, 0.0, -c], [0.0, 1.0, 0.0]])
    return mat_to_quat(m)


def mat_to_quat(m: np.ndarray) -> np.ndarray:
    """Rotation matrix -> quaternion (w, x, y, z)."""
    t = np.trace(m)
    if t > 0:
        r = math.sqrt(1.0 + t)
        w = 0.5 * r
        x = (m[2, 1] - m[1, 2]) / (2 * r)
        y = (m[0, 2] - m[2, 0]) / (2 * r)
        z = (m[1, 0] - m[0, 1]) / (2 * r)
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        r = math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k])
        q = np.empty(4)
        q[i + 1] = 0.5 * r
        q[j + 1] = (m[j, i] + m[i, j]) / (2 * r)
        q[k + 1] = (m[k, i] + m[i, k]) / (2 * r)
        q[0] = (m[k, j] - m[j, k]) / (2 * r)
        return q
    return np.array([w, x, y, z])


def quat_to_mat(q) -> np.ndarray:
    w, x, y, z = [float(v) for v in q]
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def wrist_pose_for_tip(tip_pos: np.ndarray, wrist_quat: np.ndarray):
    """Wrist position such that the fingertip center lands at tip_pos."""
    offset_w = quat_to_mat(wrist_quat) @ np.array(wc.FINGERTIP_OFFSET_IN_WRIST)
    return tip_pos - offset_w, wrist_quat


def load_kinematics():
    from curobo.kinematics import Kinematics, KinematicsCfg

    return Kinematics(KinematicsCfg.from_robot_yaml_file(wc.CUROBO_ROBOT_YML))


def fk_calibration(kin) -> dict:
    from curobo.types import JointState

    names = kin.joint_names  # active (unlocked) joints only
    q = torch.tensor(
        [[wc.DEFAULT_JOINT_POS[n] for n in names]], device="cuda", dtype=torch.float32
    )
    state = kin.compute_kinematics(JointState.from_position(q, joint_names=names))
    pose = state.tool_poses.get_link_pose(wc.LEFT_EE_BODY)
    wrist_pos = pose.position.view(-1).cpu().numpy()
    wrist_quat = pose.quaternion.view(-1).cpu().numpy()
    tip_pos = wrist_pos + quat_to_mat(wrist_quat) @ np.array(wc.FINGERTIP_OFFSET_IN_WRIST)
    print(f"active joints ({len(names)}): {names}")
    print(f"default wrist pos  (base frame): {np.round(wrist_pos, 4)}")
    print(f"default wrist quat (wxyz):       {np.round(wrist_quat, 4)}")
    print(f"default fingertip-center pos:    {np.round(tip_pos, 4)}")
    approach_w = quat_to_mat(wrist_quat) @ np.array(wc.APPROACH_AXIS_IN_WRIST)
    print(f"approach axis in base frame:     {np.round(approach_w, 4)}")
    return {"names": names, "q": q, "wrist_pos": wrist_pos, "tip_pos": tip_pos}


def run_grid(args):
    from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
    from curobo.scene import Cuboid, Scene
    from curobo.types import GoalToolPose, JointState

    cx, cy, cz = args.box_center
    dx, dy, dz = args.box_extent
    table_top = args.table_top

    # Table must not overlap the robot column at x~0: front edge at x_min.
    table_x_min = args.table_x_min
    table_dims_x = 0.9
    scene = Scene(
        cuboid=[
            Cuboid(
                name="table",
                dims=[table_dims_x, 1.2, 0.05],
                pose=[table_x_min + table_dims_x / 2, cy, table_top - 0.025,
                      1.0, 0.0, 0.0, 0.0],
            )
        ]
    )
    config = MotionPlannerCfg.create(
        robot=wc.CUROBO_ROBOT_YML,
        scene_model=scene,
        collision_cache={"cuboid": 10},
        max_goalset=args.num_yaws,
        num_ik_seeds=32,
    )
    planner = MotionPlanner(config)
    print("warming up planner ...")
    t0 = time.time()
    planner.warmup(enable_graph=False, num_warmup_iterations=2)
    print(f"warmup {time.time() - t0:.1f}s")

    names = planner.joint_names
    q_start = JointState.from_position(
        torch.tensor([[wc.DEFAULT_JOINT_POS[n] for n in names]], device="cuda"),
        joint_names=names,
    )

    # Baseline: near-default goal must plan, else the setup itself is broken
    # (e.g. start state in collision with the table).
    pose0 = planner.compute_kinematics(q_start).tool_poses.get_link_pose(wc.LEFT_EE_BODY)
    p0 = pose0.position.view(1, 1, 1, 1, 3).clone()
    p0[..., 2] += 0.05
    base_goal = GoalToolPose(
        tool_frames=planner.tool_frames,
        position=p0, quaternion=pose0.quaternion.view(1, 1, 1, 1, 4).clone(),
    )
    res0 = planner.plan_pose(base_goal, q_start.clone())
    ok0 = res0 is not None and bool(res0.success.any())
    print(f"baseline near-default plan: {'OK' if ok0 else 'FAILED'}")
    if not ok0:
        print("aborting grid: baseline failed — check start-state collisions")
        return

    yaws = [i * 2 * math.pi / args.num_yaws for i in range(args.num_yaws)]
    xs = np.linspace(cx - dx / 2, cx + dx / 2, args.grid[0])
    ys = np.linspace(cy - dy / 2, cy + dy / 2, args.grid[1])
    zs = np.linspace(cz - dz / 2, cz + dz / 2, args.grid[2])

    n_ok, n_total, times = 0, 0, []
    failures = []
    for x in xs:
        for y in ys:
            for z in zs:
                tip = np.array([x, y, z])
                pos = torch.zeros(1, 1, 1, args.num_yaws, 3, device="cuda")
                quat = torch.zeros(1, 1, 1, args.num_yaws, 4, device="cuda")
                for gi, yaw in enumerate(yaws):
                    wq = top_down_wrist_quat(yaw)
                    wp, _ = wrist_pose_for_tip(tip, wq)
                    pos[0, 0, 0, gi] = torch.tensor(wp, dtype=torch.float32)
                    quat[0, 0, 0, gi] = torch.tensor(wq, dtype=torch.float32)
                goal = GoalToolPose(
                    tool_frames=planner.tool_frames, position=pos, quaternion=quat
                )
                t0 = time.time()
                result = planner.plan_pose(goal, q_start.clone())
                dt = time.time() - t0
                ok = result is not None and bool(result.success.any())
                n_total += 1
                n_ok += int(ok)
                times.append(dt)
                if not ok:
                    failures.append((round(x, 3), round(y, 3), round(z, 3)))

    print(f"\nplan success: {n_ok}/{n_total} ({100.0 * n_ok / max(n_total, 1):.1f}%)")
    print(f"plan time: mean {np.mean(times):.3f}s  median {np.median(times):.3f}s")
    if failures:
        print(f"failed points ({len(failures)}): {failures[:20]}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fk-only", action="store_true")
    parser.add_argument("--box-center", type=float, nargs=3, default=None,
                        help="Workspace box center (base frame). Default: derived from FK.")
    parser.add_argument("--box-extent", type=float, nargs=3, default=[0.30, 0.30, 0.15])
    parser.add_argument("--table-top", type=float, default=None,
                        help="Table top z. Default: box-center z minus half extent minus 2cm.")
    parser.add_argument("--grid", type=int, nargs=3, default=[4, 4, 3])
    parser.add_argument("--num-yaws", type=int, default=4)
    parser.add_argument("--table-x-min", type=float, default=0.18,
                        help="Table front edge x (must clear the robot column at x~0).")
    args = parser.parse_args()

    kin = load_kinematics()
    calib = fk_calibration(kin)
    if args.fk_only:
        return

    if args.box_center is None:
        tip = calib["tip_pos"]
        args.box_center = [float(tip[0]), float(tip[1]), float(tip[2])]
        print(f"box center defaulted to default fingertip pos {np.round(tip, 3)}")
    if args.table_top is None:
        args.table_top = args.box_center[2] - args.box_extent[2] / 2 - 0.02
    print(f"workspace box center={args.box_center} extent={args.box_extent} "
          f"table_top={args.table_top:.3f}")
    del kin
    run_grid(args)


if __name__ == "__main__":
    main()
