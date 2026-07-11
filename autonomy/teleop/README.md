# Teleop

Quest 2 WebXR hand tracking → ROS 2. Used by the Isaac Sim bimanual teleop pipeline.

| Package | Role |
|---------|------|
| [`quest_teleop/`](quest_teleop/) | WebXR page + WSS bridge → `/quest_teleop` |

**Full setup (certs, adb, Isaac Sim):**  
[`autonomy/simulation/quest_isaac_teleop/README.md`](../simulation/quest_isaac_teleop/README.md)

**Bridge-only (node + WebXR server):**  
[`quest_teleop/README.md`](quest_teleop/README.md)
