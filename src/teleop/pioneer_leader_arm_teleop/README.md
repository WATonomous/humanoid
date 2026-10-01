# pioneer_leader_arm_teleop

The 7-servo leader arm (STS3215, torque always off) drives the Pioneer left arm in Isaac Sim,
joint to joint with no IK, in any registered scene. Optional recording in the shared
`dataset_schema_pioneer_v1.yaml` format.

| Servo | Bus ID | Sim joint | Default sign | Scale |
|-------|--------|-----------|--------------|-------|
| A | 2 | `joint1L` shoulder flexion | +1 | 0.7 |
| B | 3 | `joint2l` shoulder abduction | −1 | 0.7 |
| C | 1 | `joint3l` shoulder rotation | −1 | 0.7 |
| D | 5 | `joint4l` elbow flexion | +1 | 1.0 |
| E | 4 | `joint5l` forearm rotation | +1 | 1.0 |
| F | 7 | `joint6l` wrist | −1 | 1.0 |
| G | 6 | gripper (41.5° open → 0° closed) | +1 | — |

## Run

The leader plugs in over USB (`/dev/ttyACM0`); the `simulation_isaac` container is privileged and mounts `/dev`. On the host you need the `dialout` group (`sudo usermod -aG dialout $USER`, then log out and in).

```bash
# on the host, once: build and start the Isaac container
./watod build simulation_isaac_dev        # rebuild after pulling Dockerfile or package changes
./watod up -d
./watod -t simulation_isaac_dev           # shell inside the container

# inside the container
cd /workspace/humanoid/src/teleop/pioneer_leader_arm_teleop
$PYTHON encoder_test.py                   # first time: all 7 servos respond? (--ids 2,3 --raw for a subset)

PYTHONPATH=$(pwd) /workspace/isaaclab/isaaclab.sh -p pioneer_leader_arm_teleop.py --scene push
PYTHONPATH=$(pwd) /workspace/isaaclab/isaaclab.sh -p pioneer_leader_arm_teleop.py --scene vial_rack --record
```

Scenes: `bare` (default), `push`, `vial_rack`, or any scene registered in `humanoid_isaac_scenes` (an unknown name lists them).

- **Home pose:** hold the leader with the elbow bent like the sim arm and the gripper **open** at startup and whenever you press **R**. Leader zero maps to the sim home pose.
- **Directions:** move one leader joint at a time. If a sim joint goes the wrong way, restart with that entry flipped in `--signs` (order A..G, default `1,-1,-1,1,1,-1,1`).
- **R:** re-zero the leader and reset the arm and every object in the scene. During a take, it also discards the take and recording restarts from home.
- **`--record`:** `S` start · `N` save (then auto-reset) · `D` discard → `<repo>/datasets/pioneer_v1_left_arm/sim/` · `--cameras ego,wrist_left` / `none`.
- Other flags: `--port`, `--baud`, `--scale` (overall gain), `--filter-alpha` (target smoothing, default 0.35).

Targets are clamped to the arm's URDF limits. Wrist damping is lowered to 2.5 in this script only, so the sim wrist keeps up with the leader.
