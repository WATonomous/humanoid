# Projects Not Yet On Resume

Use this file as a source of polished bullets and interview stories for robotics roles. The wording keeps metrics where available and leaves placeholders where the exact number should be measured later.

## Project 1: Quest 2 Teleop For Wato Sim

### Resume Bullets

- Built a Meta Quest 2 to ROS2 teleoperation pipeline for humanoid hand control, streaming 25 WebXR joints per hand plus wrist poses over secure WebSockets into `/quest_teleop` at ~90 Hz for live Isaac Lab simulation experiments.
- Implemented an end-to-end Quest teleop stack with HTTPS WebXR hand tracking, a C++ ROS2 WSS bridge, a custom `QuestHandPose` message, and a Python Isaac Lab subscriber for real-time Wato arm/hand control prototyping.
- Added in-headset hand landmark visualization to validate Quest tracking quality before debugging robot retargeting, reducing ambiguity between sensor, transport, and simulation-control issues.

### STAR

**Situation:** The humanoid project needed an inexpensive real-time teleoperation interface for simulated hands and arms.

**Task:** Stream Quest 2 hand tracking into ROS2 and make it usable inside Isaac Lab.

**Action:** Built a WebXR client, HTTPS server, C++ WSS-to-ROS2 bridge, custom ROS message, and Isaac Lab subscriber/controller. Added colored hand points in the headset for tracking validation.

**Result:** Achieved ~90 Hz hand-tracking transport into ROS2, carrying 25 joints per hand, 75 xyz floats per hand, and wrist poses for both hands.

### Key Technologies

```text
Meta Quest 2, WebXR, HTTPS/WSS, C++, ROS2, custom ROS messages, Isaac Lab, Python, humanoid teleoperation
```

### Files

```text
autonomy/teleop/quest_teleop/static/index.html
autonomy/teleop/quest_teleop/src/quest_teleop_node.cpp
autonomy/teleop/quest_teleop/msg/QuestHandPose.msg
autonomy/teleop/quest_sim_teleop/quest_sim_teleop/quest_live_arm_teleop.py
```

## Project 2: Sim2Real Manipulation Data Pipeline

### Resume Bullets

- Built a sim-to-real manipulation data pipeline by recreating a measured real-world room in simulation, converting robot assets from URDF to USD, tuning collision geometry, collecting 40 simulated demonstrations, and fine-tuning a GR00T N1.6 policy with additional real demonstrations.
- Recreated a real manipulation workspace in Isaac Sim using measured room-scale dimensions, calibrated camera placement, URDF-to-USD robot import, convex decomposition for the gripper, and convex hull collision meshes for remaining robot/environment assets.
- Implemented leader-follower teleoperation by mapping real leader-arm motor states to a follower arm in simulation, enabling demonstration collection in a fixed environment and under domain randomization.
- Collected 30 unchanged-environment demonstrations, 10 domain-randomized demonstrations, and 5 real-world demonstrations, then used Cosmos Transfer 2.5 for sim-to-photoreal data generation before fine-tuning GR00T N1.6.
- Deployed GR00T training/inference infrastructure using a GR00T server on Brev to support remote fine-tuning workflows for sim2real manipulation experiments.

### STAR

**Situation:** The manipulation project needed a way to transfer skills from simulation to the real robot with limited real-world data.

**Task:** Build a realistic simulation environment, collect demonstrations, improve visual realism, and fine-tune a robot foundation model for the task.

**Action:** Measured the real room to scale and replicated it in simulation, using a manager-based/environment-based setup as appropriate for the task structure. Set up camera placement, converted robot assets from URDF to USD, used convex decomposition for the gripper, and used convex hull collision approximations for the rest of the assets. Mapped real leader-arm motor readings to a simulated follower arm for teleoperation and demonstration collection. Collected demonstrations in both unchanged and randomized environments, used Cosmos Transfer 2.5 for photorealistic sim transfer, collected a small set of real demonstrations, and fine-tuned GR00T N1.6 using a GR00T server hosted on Brev.

**Result:** Built a full sim2real data pipeline with 45 total demonstrations across sim and real data: 30 fixed-environment sim demos, 10 domain-randomized sim demos, and 5 real demos. Produced a dataset and fine-tuning workflow for evaluating sim-to-real manipulation transfer.

### Key Metrics

```text
30 fixed-environment simulated demonstrations
10 domain-randomized simulated demonstrations
5 real-world demonstrations
45 total demonstrations
URDF -> USD asset conversion
Cosmos Transfer 2.5 photorealism pass
GR00T N1.6 fine-tuning
GR00T server hosted on Brev
```

### Key Technologies

```text
Isaac Sim / Isaac Lab, URDF, USD, collision geometry, convex decomposition, convex hulls, teleoperation, domain randomization, Cosmos Transfer 2.5, GR00T N1.6, Brev
```

### Interview Talking Points

- Measuring the room mattered because sim2real transfer depends on camera viewpoints, object placement, robot reachability, and scene scale.
- Collision geometry mattered because visual meshes are usually too complex or unstable for physics; convex decomposition was used where gripper contact fidelity mattered most, while convex hulls were sufficient for simpler geometry.
- Keeping 30 demonstrations in the unchanged environment gave a clean baseline distribution.
- Adding 10 randomized demonstrations helped expose the policy to variation before real deployment.
- Adding 5 real demonstrations anchored the fine-tuning set to real sensor/robot behavior.
- Cosmos Transfer 2.5 helped bridge the visual gap between synthetic simulation and real imagery.
- GR00T N1.6 was used as the policy/foundation-model fine-tuning target.

### Short Interview Answer

I built a sim2real manipulation pipeline where I recreated the physical workspace in simulation from room-scale measurements, converted robot assets from URDF to USD, simplified collision geometry for stable physics, and mapped a real leader arm to a simulated follower arm for teleoperated data collection. I collected 30 fixed-environment sim demonstrations, 10 domain-randomized sim demonstrations, and 5 real demonstrations. Then I used Cosmos Transfer 2.5 to improve sim photorealism and fine-tuned GR00T N1.6 with a GR00T server running on Brev.

## Combined Resume Section Draft

### Robotics Teleoperation And Sim2Real Projects

- Built a Meta Quest 2 teleoperation pipeline for humanoid simulation, streaming 25 WebXR joints per hand plus wrist poses into ROS2 at ~90 Hz through an HTTPS/WSS browser client, C++ ROS2 bridge, and Isaac Lab subscriber.
- Developed a sim2real manipulation data pipeline by recreating a measured real-world workspace in Isaac Sim, converting URDF robot assets to USD, tuning collision geometry, and collecting 45 demonstrations across fixed sim, randomized sim, and real-world settings.
- Fine-tuned GR00T N1.6 for manipulation using sim and real demonstrations, Cosmos Transfer 2.5 photorealistic sim data, and a GR00T server deployed on Brev.

## Needs Verification Before Resume

- Exact task name for the sim2real manipulation project.
- Exact robot/arm names for leader and follower.
- Whether the sim environment was primarily manager-based, direct/environment-based, or a mix.
- Final policy success rate, if measured.
- Any latency numbers for Quest teleop beyond topic rate.
- Whether GR00T N1.6 fine-tuning completed successfully or is still in progress.

