Yes, right now we’re only using the 75 hand landmark points to drive finger curl. The wrist pose is being published too, but `quest_Isaac.py` is not using it yet.

Your message has:

```text
geometry_msgs/Pose left_wrist
geometry_msgs/Pose right_wrist
float32[] left_hand_joints
float32[] right_hand_joints
```

To control wrist/arm pose, you need a frame transform pipeline.

**Big Picture**

You want:

```text
Quest wrist pose
  -> Quest/world coordinate frame
  -> robot/sim world frame
  -> robot base frame
  -> desired wrist/hand target
  -> arm joint targets through IK
```

So there are two separate problems:

1. **Coordinate transform**: where is the Quest wrist relative to the robot?
2. **Inverse kinematics**: what shoulder/elbow/wrist joint angles make the robot wrist reach that target?

**Step 1: Understand Pose As Transform**

A wrist pose is position + orientation:

```text
position: x, y, z
orientation: quaternion x, y, z, w
```

You can think of it as a 4x4 transform matrix:

```text
T_world_wrist =
[ R  p ]
[ 0  1 ]
```

where:

- `R` is 3x3 rotation from quaternion
- `p` is 3D position

**Step 2: Calibrate Quest Frame To Robot Frame**

You need a transform like:

```text
T_robotWorld_questWorld
```

This says: “given a pose in Quest coordinates, convert it into Isaac/robot world coordinates.”

Then:

```text
T_robotWorld_questWrist =
T_robotWorld_questWorld @ T_questWorld_questWrist
```

In code/math terms:

```python
p_robot = R_calib @ p_quest + t_calib
R_robot = R_calib @ R_quest
```

where:

- `R_calib` rotates Quest axes into Isaac axes
- `t_calib` shifts the Quest origin to the robot origin

**Step 3: Convert World Wrist Target To Robot Base Frame**

IK usually wants target pose relative to the robot base.

If you know:

```text
T_world_base
T_world_wrist_target
```

then:

```text
T_base_wrist_target = inverse(T_world_base) @ T_world_wrist_target
```

That gives the wrist goal in the robot’s base frame.

**Step 4: Use IK**

Once you have desired wrist pose in robot base frame, don’t manually solve all arm joints at first. Use IsaacLab IK tools or a differential IK controller.

Conceptually:

```text
desired wrist pose -> IK -> shoulder/elbow/forearm/wrist joint targets
```

Then fingers continue using your current curl mapping.

**What You Should Implement First**

Start with position only. Ignore orientation at first.

1. Print Quest wrist:

```python
msg.left_wrist.position
msg.left_wrist.orientation
```

2. Pick a neutral calibration:
   - Put your Quest hand in a known pose.
   - Put robot wrist in matching known pose.
   - Record both.
   - Compute offset:

```python
t_calib = robot_neutral_position - quest_neutral_position
```

3. Apply simple translation:

```python
target_pos = quest_pos + t_calib
```

4. Then add axis remapping.

Quest and Isaac probably disagree on axes. For example, you may discover something like:

```python
isaac_x = -quest_z
isaac_y = -quest_x
isaac_z = quest_y
```

That is your `R_calib`.

**Useful Mental Model**

Every transform answers:

```text
Where is frame B, expressed in frame A?
```

Written as:

```text
T_A_B
```

So:

```text
T_questWorld_questWrist
```

means wrist pose expressed in Quest world.

```text
T_isaacWorld_questWorld
```

means Quest world expressed in Isaac world.

Then:

```text
T_isaacWorld_questWrist =
T_isaacWorld_questWorld @ T_questWorld_questWrist
```

The middle frames cancel:

```text
isaacWorld <- questWorld <- questWrist
```

**Practical Roadmap**

1. Keep fingers working.
2. Visualize wrist target as a small sphere in Isaac.
3. Move your real wrist and confirm the sphere moves correctly.
4. Add calibration offset/axis remap until motion feels correct.
5. Use IK to make robot wrist follow the sphere.
6. Add wrist orientation later.
7. Tune smoothing/latency.

The wrist math is mostly transforms. The arm motion is IK. Don’t mix them too early or it gets confusing fast.