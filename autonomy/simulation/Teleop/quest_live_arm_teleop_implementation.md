# Quest Live Arm Teleop Implementation

Goal: create one new sim-side Python node that subscribes to `/quest_teleop`, converts Quest WebXR hand landmarks into Wato arm/hand joint targets, and applies those targets live inside Isaac Sim.

We will not reuse the dex-retargeting code. This implementation is simple geometry:

```text
Quest 25 hand landmarks
  -> finger bone vectors
  -> mcp/pip/dip/thumb bend angles
  -> clamp to Wato URDF limits
  -> Isaac set_joint_position_target(...)
```

## New File

Create:

```text
autonomy/simulation/Teleop/quest_live_arm_teleop.py
```

This file is both:

```text
1. The Isaac Sim app/script
2. The ROS2 subscriber node for /quest_teleop
```

That keeps the design to one node/process for the live sim teleop path.

## Existing Input

The Quest teleop bridge publishes:

```text
topic: /quest_teleop
type:  quest_teleop/msg/QuestHandPose
```

Message:

```text
geometry_msgs/Pose left_wrist
geometry_msgs/Pose right_wrist
float32[] left_hand_joints
float32[] right_hand_joints
```

The hand arrays are:

```text
25 joints * xyz = 75 floats
```

The order comes from `autonomy/teleop/quest_teleop/static/index.html`:

```python
WEBXR_JOINTS = [
    "wrist",
    "thumb-metacarpal",
    "thumb-phalanx-proximal",
    "thumb-phalanx-distal",
    "thumb-tip",
    "index-finger-metacarpal",
    "index-finger-phalanx-proximal",
    "index-finger-phalanx-intermediate",
    "index-finger-phalanx-distal",
    "index-finger-tip",
    "middle-finger-metacarpal",
    "middle-finger-phalanx-proximal",
    "middle-finger-phalanx-intermediate",
    "middle-finger-phalanx-distal",
    "middle-finger-tip",
    "ring-finger-metacarpal",
    "ring-finger-phalanx-proximal",
    "ring-finger-phalanx-intermediate",
    "ring-finger-phalanx-distal",
    "ring-finger-tip",
    "pinky-finger-metacarpal",
    "pinky-finger-phalanx-proximal",
    "pinky-finger-phalanx-intermediate",
    "pinky-finger-phalanx-distal",
    "pinky-finger-tip",
]
```

## Sim Output

Each arm has 21 DOF:

```text
6 arm joints + 15 hand joints
```

Right arm joint order from `ARM_CFG`:

```python
RIGHT_JOINTS = [
    "shoulder_flexion_extension",
    "shoulder_abduction_adduction",
    "shoulder_rotation",
    "elbow_flexion_extension",
    "forearm_rotation",
    "wrist_extension",
    "mcp_index",
    "pip_index",
    "dip_index",
    "mcp_middle",
    "pip_middle",
    "dip_middle",
    "mcp_ring",
    "pip_ring",
    "dip_ring",
    "mcp_pinky",
    "pip_pinky",
    "dip_pinky",
    "cmc_thumb",
    "mcp_thumb",
    "ip_thumb",
]
```

Left arm joint order from `LEFT_ARM_CFG`:

```python
LEFT_JOINTS = [
    "left_shoulder_flexion_extension",
    "left_shoulder_abduction_adduction",
    "left_shoulder_rotation",
    "left_elbow_flexion_extension",
    "left_forearm_rotation",
    "left_wrist_extension",
    "left_mcp_index",
    "left_pip_index",
    "left_dip_index",
    "left_mcp_middle",
    "left_pip_middle",
    "left_dip_middle",
    "left_mcp_ring",
    "left_pip_ring",
    "left_dip_ring",
    "left_mcp_pinky",
    "left_pip_pinky",
    "left_dip_pinky",
    "left_cmc_thumb",
    "left_mcp_thumb",
    "left_ip_thumb",
]
```

## Joint Limits

Use these limits for both left and right. They match the arm assembly URDF files.

```python
JOINT_LIMITS = {
    "wrist_extension": (-0.959931, 0.959931),
    "mcp_index": (-1.570796, 0.0),
    "pip_index": (0.0, 1.570796),
    "dip_index": (-1.570796, 0.0),
    "mcp_middle": (-1.570796, 0.0),
    "pip_middle": (0.0, 1.570796),
    "dip_middle": (0.0, 1.570796),
    "mcp_ring": (0.0, 1.570796),
    "pip_ring": (-1.570796, 0.0),
    "dip_ring": (-1.570796, 0.0),
    "mcp_pinky": (0.0, 1.570796),
    "pip_pinky": (-1.570796, 0.0),
    "dip_pinky": (0.0, 1.570796),
    "cmc_thumb": (-0.349066, 2.094395),
    "mcp_thumb": (0.785398, 2.530727),
    "ip_thumb": (0.0, 1.570796),
}
```

For left joints, strip the `left_` prefix before looking up limits:

```python
base_name = joint_name.removeprefix("left_")
lower, upper = JOINT_LIMITS[base_name]
```

## Angle Math

Convert the flat 75-float array into named xyz points:

```python
def hand_array_to_points(values):
    points = {}
    for i, name in enumerate(WEBXR_JOINTS):
        j = 3 * i
        points[name] = np.array(values[j:j + 3], dtype=np.float32)
    return points
```

Compute a bend angle between two adjacent bone vectors:

```python
def vector_angle(a, b):
    a_norm = np.linalg.norm(a)
    b_norm = np.linalg.norm(b)
    if a_norm < 1e-6 or b_norm < 1e-6:
        return 0.0

    a = a / a_norm
    b = b / b_norm
    dot = np.clip(np.dot(a, b), -1.0, 1.0)
    return float(np.arccos(dot))
```

For one normal finger:

```python
def finger_angles(points, prefix):
    metacarpal = points[f"{prefix}-finger-metacarpal"]
    proximal = points[f"{prefix}-finger-phalanx-proximal"]
    intermediate = points[f"{prefix}-finger-phalanx-intermediate"]
    distal = points[f"{prefix}-finger-phalanx-distal"]
    tip = points[f"{prefix}-finger-tip"]

    v0 = proximal - metacarpal
    v1 = intermediate - proximal
    v2 = distal - intermediate
    v3 = tip - distal

    mcp = vector_angle(v0, v1)
    pip = vector_angle(v1, v2)
    dip = vector_angle(v2, v3)
    return mcp, pip, dip
```

For the thumb:

```python
def thumb_angles(points):
    wrist = points["wrist"]
    metacarpal = points["thumb-metacarpal"]
    proximal = points["thumb-phalanx-proximal"]
    distal = points["thumb-phalanx-distal"]
    tip = points["thumb-tip"]

    v0 = metacarpal - wrist
    v1 = proximal - metacarpal
    v2 = distal - proximal
    v3 = tip - distal

    cmc = vector_angle(v0, v1)
    mcp = vector_angle(v1, v2)
    ip = vector_angle(v2, v3)
    return cmc, mcp, ip
```

The raw bend angle is always positive. The robot joints use mixed signs, so convert a positive curl angle into each joint's legal signed range:

```python
def map_curl_to_joint(curl, joint_name):
    base_name = joint_name.removeprefix("left_")
    lower, upper = JOINT_LIMITS[base_name]

    curl = float(np.clip(curl, 0.0, 1.570796))

    if upper <= 0.0:
        value = -curl
    elif lower >= 0.0:
        value = curl
    else:
        value = curl

    return float(np.clip(value, lower, upper))
```

Thumb MCP has a nonzero lower limit, so offset it from its open pose:

```python
def map_thumb_mcp(curl, joint_name):
    base_name = joint_name.removeprefix("left_")
    lower, upper = JOINT_LIMITS[base_name]
    value = lower + float(np.clip(curl, 0.0, 1.570796))
    return float(np.clip(value, lower, upper))
```

## Hand Target Function

This function returns the 15 hand joint targets for one side.

```python
def compute_hand_targets(points, side):
    prefix = "left_" if side == "left" else ""

    targets = {}

    for finger in ["index", "middle", "ring", "pinky"]:
        mcp, pip, dip = finger_angles(points, finger)
        targets[f"{prefix}mcp_{finger}"] = map_curl_to_joint(mcp, f"{prefix}mcp_{finger}")
        targets[f"{prefix}pip_{finger}"] = map_curl_to_joint(pip, f"{prefix}pip_{finger}")
        targets[f"{prefix}dip_{finger}"] = map_curl_to_joint(dip, f"{prefix}dip_{finger}")

    cmc, mcp, ip = thumb_angles(points)
    targets[f"{prefix}cmc_thumb"] = map_curl_to_joint(cmc, f"{prefix}cmc_thumb")
    targets[f"{prefix}mcp_thumb"] = map_thumb_mcp(mcp, f"{prefix}mcp_thumb")
    targets[f"{prefix}ip_thumb"] = map_curl_to_joint(ip, f"{prefix}ip_thumb")

    return targets
```

## ROS Subscriber Class

The node stores the latest message. The sim loop calls `rclpy.spin_once(node, timeout_sec=0.0)` every frame.

```python
class QuestTeleopSubscriber(Node):
    def __init__(self):
        super().__init__("quest_live_arm_teleop")
        self.latest_msg = None
        self.last_msg_time = None
        self.create_subscription(
            QuestHandPose,
            "/quest_teleop",
            self._on_msg,
            1,
        )

    def _on_msg(self, msg):
        self.latest_msg = msg
        self.last_msg_time = time.monotonic()
```

## Sim Loop Behavior

In `run_simulator(...)`:

```text
1. Resolve right and left arm joint ids.
2. Keep target tensors initialized at default joint positions.
3. Every sim frame:
   - spin ROS once
   - if a recent Quest message exists:
     - compute right hand targets from right_hand_joints
     - compute left hand targets from left_hand_joints
     - write those values into the target tensors
   - send targets with set_joint_position_target(...)
   - scene.write_data_to_sim()
   - sim.step()
   - scene.update(sim_dt)
```

Timeout behavior:

```python
MESSAGE_TIMEOUT_SEC = 0.25
```

If no message has arrived for 0.25 seconds, hold the last target or slowly return to default. For the first version, hold last target.

## Full Code Skeleton

Paste this into:

```text
autonomy/simulation/Teleop/quest_live_arm_teleop.py
```

```python
import argparse
import time

import numpy as np
import torch

import rclpy
from rclpy.node import Node
from quest_teleop.msg import QuestHandPose

import isaaclab.sim as sim_utils
from isaaclab.app import AppLauncher
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass

from HumanoidRL.HumanoidRLPackage.HumanoidRLSetup.modelCfg.humanoid import (
    ARM_CFG,
    LEFT_ARM_CFG,
)


parser = argparse.ArgumentParser(description="Quest live teleop for Wato arms")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


WEBXR_JOINTS = [
    "wrist",
    "thumb-metacarpal",
    "thumb-phalanx-proximal",
    "thumb-phalanx-distal",
    "thumb-tip",
    "index-finger-metacarpal",
    "index-finger-phalanx-proximal",
    "index-finger-phalanx-intermediate",
    "index-finger-phalanx-distal",
    "index-finger-tip",
    "middle-finger-metacarpal",
    "middle-finger-phalanx-proximal",
    "middle-finger-phalanx-intermediate",
    "middle-finger-phalanx-distal",
    "middle-finger-tip",
    "ring-finger-metacarpal",
    "ring-finger-phalanx-proximal",
    "ring-finger-phalanx-intermediate",
    "ring-finger-phalanx-distal",
    "ring-finger-tip",
    "pinky-finger-metacarpal",
    "pinky-finger-phalanx-proximal",
    "pinky-finger-phalanx-intermediate",
    "pinky-finger-phalanx-distal",
    "pinky-finger-tip",
]


JOINT_LIMITS = {
    "wrist_extension": (-0.959931, 0.959931),
    "mcp_index": (-1.570796, 0.0),
    "pip_index": (0.0, 1.570796),
    "dip_index": (-1.570796, 0.0),
    "mcp_middle": (-1.570796, 0.0),
    "pip_middle": (0.0, 1.570796),
    "dip_middle": (0.0, 1.570796),
    "mcp_ring": (0.0, 1.570796),
    "pip_ring": (-1.570796, 0.0),
    "dip_ring": (-1.570796, 0.0),
    "mcp_pinky": (0.0, 1.570796),
    "pip_pinky": (-1.570796, 0.0),
    "dip_pinky": (0.0, 1.570796),
    "cmc_thumb": (-0.349066, 2.094395),
    "mcp_thumb": (0.785398, 2.530727),
    "ip_thumb": (0.0, 1.570796),
}


@configclass
class QuestBothArmsSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )

    right_arm = ARM_CFG.replace(
        prim_path="{ENV_REGEX_NS}/RightArm",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.5, 0.0, 0.0),
            joint_pos=ARM_CFG.init_state.joint_pos,
        ),
    )

    left_arm = LEFT_ARM_CFG.replace(
        prim_path="{ENV_REGEX_NS}/LeftArm",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(-0.5, 0.0, 0.0),
            joint_pos=LEFT_ARM_CFG.init_state.joint_pos,
        ),
    )


class QuestTeleopSubscriber(Node):
    def __init__(self):
        super().__init__("quest_live_arm_teleop")
        self.latest_msg = None
        self.last_msg_time = None
        self.create_subscription(QuestHandPose, "/quest_teleop", self._on_msg, 1)

    def _on_msg(self, msg):
        self.latest_msg = msg
        self.last_msg_time = time.monotonic()


def hand_array_to_points(values):
    if len(values) != 75:
        return None

    points = {}
    for i, name in enumerate(WEBXR_JOINTS):
        j = 3 * i
        points[name] = np.array(values[j:j + 3], dtype=np.float32)
    return points


def vector_angle(a, b):
    a_norm = np.linalg.norm(a)
    b_norm = np.linalg.norm(b)
    if a_norm < 1e-6 or b_norm < 1e-6:
        return 0.0

    a = a / a_norm
    b = b / b_norm
    dot = np.clip(np.dot(a, b), -1.0, 1.0)
    return float(np.arccos(dot))


def finger_angles(points, prefix):
    metacarpal = points[f"{prefix}-finger-metacarpal"]
    proximal = points[f"{prefix}-finger-phalanx-proximal"]
    intermediate = points[f"{prefix}-finger-phalanx-intermediate"]
    distal = points[f"{prefix}-finger-phalanx-distal"]
    tip = points[f"{prefix}-finger-tip"]

    v0 = proximal - metacarpal
    v1 = intermediate - proximal
    v2 = distal - intermediate
    v3 = tip - distal

    return vector_angle(v0, v1), vector_angle(v1, v2), vector_angle(v2, v3)


def thumb_angles(points):
    wrist = points["wrist"]
    metacarpal = points["thumb-metacarpal"]
    proximal = points["thumb-phalanx-proximal"]
    distal = points["thumb-phalanx-distal"]
    tip = points["thumb-tip"]

    v0 = metacarpal - wrist
    v1 = proximal - metacarpal
    v2 = distal - proximal
    v3 = tip - distal

    return vector_angle(v0, v1), vector_angle(v1, v2), vector_angle(v2, v3)


def clamp_to_limit(value, joint_name):
    base_name = joint_name.removeprefix("left_")
    lower, upper = JOINT_LIMITS[base_name]
    return float(np.clip(value, lower, upper))


def map_curl_to_joint(curl, joint_name):
    base_name = joint_name.removeprefix("left_")
    lower, upper = JOINT_LIMITS[base_name]

    curl = float(np.clip(curl, 0.0, 1.570796))
    if upper <= 0.0:
        value = -curl
    else:
        value = curl

    return float(np.clip(value, lower, upper))


def map_thumb_mcp(curl, joint_name):
    base_name = joint_name.removeprefix("left_")
    lower, upper = JOINT_LIMITS[base_name]
    value = lower + float(np.clip(curl, 0.0, 1.570796))
    return float(np.clip(value, lower, upper))


def compute_hand_targets(points, side):
    prefix = "left_" if side == "left" else ""
    targets = {}

    for finger in ["index", "middle", "ring", "pinky"]:
        mcp, pip, dip = finger_angles(points, finger)
        targets[f"{prefix}mcp_{finger}"] = map_curl_to_joint(mcp, f"{prefix}mcp_{finger}")
        targets[f"{prefix}pip_{finger}"] = map_curl_to_joint(pip, f"{prefix}pip_{finger}")
        targets[f"{prefix}dip_{finger}"] = map_curl_to_joint(dip, f"{prefix}dip_{finger}")

    cmc, mcp, ip = thumb_angles(points)
    targets[f"{prefix}cmc_thumb"] = map_curl_to_joint(cmc, f"{prefix}cmc_thumb")
    targets[f"{prefix}mcp_thumb"] = map_thumb_mcp(mcp, f"{prefix}mcp_thumb")
    targets[f"{prefix}ip_thumb"] = map_curl_to_joint(ip, f"{prefix}ip_thumb")

    return targets


def apply_targets(target_tensor, robot, target_dict):
    for joint_name, value in target_dict.items():
        joint_index = robot.joint_names.index(joint_name)
        target_tensor[0, joint_index] = value


def run_simulator(sim, scene, node):
    right_arm = scene["right_arm"]
    left_arm = scene["left_arm"]

    right_cfg = SceneEntityCfg("right_arm", joint_names=[".*"], body_names=[".*"])
    left_cfg = SceneEntityCfg("left_arm", joint_names=[".*"], body_names=[".*"])
    right_cfg.resolve(scene)
    left_cfg.resolve(scene)

    right_target = right_arm.data.default_joint_pos.clone()
    left_target = left_arm.data.default_joint_pos.clone()

    sim_dt = sim.get_physics_dt()
    timeout_sec = 0.25

    while simulation_app.is_running():
        rclpy.spin_once(node, timeout_sec=0.0)

        msg = node.latest_msg
        fresh = node.last_msg_time is not None and (time.monotonic() - node.last_msg_time) < timeout_sec

        if msg is not None and fresh:
            right_points = hand_array_to_points(msg.right_hand_joints)
            left_points = hand_array_to_points(msg.left_hand_joints)

            if right_points is not None:
                right_hand_targets = compute_hand_targets(right_points, side="right")
                apply_targets(right_target, right_arm, right_hand_targets)

            if left_points is not None:
                left_hand_targets = compute_hand_targets(left_points, side="left")
                apply_targets(left_target, left_arm, left_hand_targets)

        right_arm.set_joint_position_target(right_target, joint_ids=right_cfg.joint_ids)
        left_arm.set_joint_position_target(left_target, joint_ids=left_cfg.joint_ids)

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)


def main():
    rclpy.init()
    node = QuestTeleopSubscriber()

    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.0], [0.0, 0.0, 0.0])

    scene_cfg = QuestBothArmsSceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    print("[INFO]: Quest live arm teleop ready. Waiting for /quest_teleop...")

    try:
        run_simulator(sim, scene, node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
    simulation_app.close()
```

## Commands

Build and source the teleop ROS package so the Python sim can import `quest_teleop.msg`:

```bash
cd /home/hassaan/Wato/humanoid/autonomy/teleop
colcon build --packages-select quest_teleop
source install/setup.bash
```

Terminal 1: start the Quest WSS ROS bridge:

```bash
cd /home/hassaan/Wato/humanoid/autonomy/teleop
source install/setup.bash
ros2 run quest_teleop quest_teleop_node
```

Terminal 2: serve the WebXR page:

```bash
cd /home/hassaan/Wato/humanoid/autonomy/teleop
source install/setup.bash
python3 quest_teleop/scripts/webxr_server.py
```

Terminal 3: run the Isaac sim live teleop script:

```bash
cd /home/hassaan/Wato/humanoid
source autonomy/teleop/install/setup.bash
PYTHONPATH=/home/hassaan/Wato/humanoid/autonomy/simulation/Humanoid_Wato:$PYTHONPATH \
/home/hassaan/IsaacLab/isaaclab.sh -p autonomy/simulation/Teleop/quest_live_arm_teleop.py
```

On the Quest browser, open:

```text
https://<computer-ip>:8443
```

Then press Start. The browser connects to:

```text
wss://<computer-ip>:9090
```

## Debug Commands

Check that the Quest bridge is publishing:

```bash
ros2 topic list | grep quest
ros2 topic hz /quest_teleop
ros2 topic echo /quest_teleop --once
```

If the sim cannot import `quest_teleop.msg`, the teleop workspace is not sourced in the terminal running Isaac:

```bash
source /home/hassaan/Wato/humanoid/autonomy/teleop/install/setup.bash
```

## First Tuning Pass

The first run will probably need sign tuning. Do this in this order:

```text
1. Test one hand only.
2. Print computed targets for index finger.
3. Open/close index finger in Quest.
4. If the sim bends backward, flip the sign mapping for that joint.
5. Repeat for middle/ring/pinky/thumb.
```

Add temporary prints:

```python
print(right_hand_targets)
```

or only:

```python
print({
    key: right_hand_targets[key]
    for key in ["mcp_index", "pip_index", "dip_index"]
})
```

## Wrist And Arm Pose

This first implementation should control fingers/thumb live.

The wrist pose in `/quest_teleop` is still useful, but mapping a 6D wrist pose to shoulder/elbow/forearm/wrist requires IK. Add that after the hand works.

Next step after finger teleop:

```text
Quest wrist pose
  -> target palm pose in sim frame
  -> IK over shoulder/elbow/forearm/wrist
  -> set first 6 arm joint targets
```

Do not mix IK into the first implementation. Get the hand angle path stable first, then add wrist/arm IK into the same node.
