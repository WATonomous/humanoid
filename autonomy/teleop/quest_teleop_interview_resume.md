# Quest 2 Teleop Interview And Resume Notes

## Resume Bullet

- Built a Meta Quest 2 teleoperation pipeline that streams WebXR hand tracking into ROS2 and Isaac Sim, using an HTTPS/WSS browser client, a C++ ROS2 bridge, and a Python Isaac Lab subscriber to publish 90 Hz hand-tracking data for real-time humanoid arm/hand teleop prototyping.

## Stronger Metric Version

- Built a Meta Quest 2 to ROS2 teleoperation pipeline for humanoid hand control, streaming 25 WebXR joints per hand plus wrist poses over secure WebSockets into `/quest_teleop` at ~90 Hz and retargeting the data inside Isaac Lab for live Wato arm/hand simulation experiments.

## STAR Version

### Situation

The robot needed an affordable real-time teleoperation interface for controlling simulated humanoid arms and hands. The available Quest 2 headset could provide hand tracking, but the data had to be moved from a browser environment into ROS2 and then into Isaac Lab.

### Task

Create an end-to-end pipeline that captures Quest hand tracking, streams it reliably into ROS2, and makes the data usable for live simulation control.

### Action

- Built a WebXR browser client in `quest_teleop/static/index.html` to read 25 hand joints per hand and wrist poses from the Quest.
- Served the browser client over HTTPS using `webxr_server.py` because WebXR hand tracking requires a secure browser context.
- Implemented a C++ ROS2 WebSocket bridge in `quest_teleop_node.cpp` / `wss_server.cpp` that receives JSON over WSS on port `9090`.
- Defined `QuestHandPose.msg` to package left/right wrist poses and flattened 75-float hand joint arrays.
- Published the live stream on `/quest_teleop`.
- Added an Isaac Lab Python teleop script in `quest_sim_teleop/quest_live_arm_teleop.py` that subscribes to `/quest_teleop` and maps incoming hand landmarks toward Wato arm/hand simulation targets.
- Added in-headset point visualization to debug hand tracking quality before retargeting.

### Result

- Achieved live Quest hand-tracking streaming into ROS2 at about 90 Hz.
- Streamed 25 joints per hand, or 75 xyz values per hand, plus left/right wrist pose data.
- Created a working bridge from Quest browser APIs to ROS2 and Isaac Lab simulation.
- Identified the next control-quality improvement: replacing direct angle mapping with a proper retargeting layer using palm-local coordinates, smoothing, joint limits, and later wrist/arm IK.

## One-Minute Interview Answer

I built a Quest 2 teleoperation pipeline for humanoid arm and hand simulation. The Quest browser uses WebXR to track 25 joints per hand plus wrist pose, then streams that data over secure WebSockets because WebXR requires HTTPS/WSS. On the robot side, a C++ ROS2 node receives the JSON stream, parses it into a custom `QuestHandPose` message, and publishes it on `/quest_teleop`. Then an Isaac Lab Python script subscribes to that topic and uses the incoming landmarks to drive the Wato simulated hands. I also added colored hand-point visualization inside the headset so we could verify the tracking data before debugging robot retargeting. The pipeline currently streams at around 90 Hz, and the next step is improving retargeting from raw landmarks to stable robot joint targets and wrist IK.

## Architecture

```text
Quest Browser
  -> WebXR hand tracking
  -> HTTPS page on :8443
  -> WSS JSON stream on :9090
  -> C++ ROS2 quest_teleop_node
  -> QuestHandPose.msg
  -> /quest_teleop
  -> Python Isaac Lab subscriber
  -> Wato arm/hand simulation targets
```

## Key Files

```text
autonomy/teleop/quest_teleop/static/index.html
```

Quest browser app. Reads hand tracking, draws colored point visualization, and sends JSON over WSS.

```text
autonomy/teleop/quest_teleop/scripts/webxr_server.py
```

HTTPS server for serving the WebXR page to the Quest on port `8443`.

```text
autonomy/teleop/quest_teleop/src/quest_teleop_node.cpp
```

C++ ROS2 node that starts the WSS server, parses incoming Quest messages, and publishes `/quest_teleop`.

```text
autonomy/teleop/quest_teleop/msg/QuestHandPose.msg
```

ROS2 message with left/right wrist poses and left/right flattened hand landmark arrays.

```text
autonomy/teleop/quest_sim_teleop/quest_sim_teleop/quest_live_arm_teleop.py
```

Isaac Lab script that subscribes to `/quest_teleop` and applies live simulation targets.

## Metrics To Mention

- Stream rate observed: approximately `90 Hz`.
- Hand landmark payload: `25 joints/hand * 3 xyz = 75 floats/hand`.
- Both hands: `150 hand-coordinate floats/frame`.
- Wrist pose payload: position xyz plus quaternion xyzw for each wrist.
- Transport: HTTPS page on `8443`, WSS stream on `9090`, ROS2 topic `/quest_teleop`.

## Interview Talking Points

- Why WebXR: browser-native hand tracking on Quest, no native Quest app required.
- Why HTTPS/WSS: WebXR hand tracking requires a secure context.
- Why C++ bridge: low-latency ROS2 transport and robust WebSocket parsing.
- Why Python sim node: Isaac Lab is Python-first, so simulation control is cleaner in Python.
- Why retargeting is hard: Quest landmarks are not robot joints; robot joint limits, signs, axes, and kinematics differ from the human hand.
- What needs improvement: stable landmark-to-joint retargeting, smoothing, timeout handling, workspace limits, and wrist/arm IK.

## Honest Current Status

The pipeline successfully streams Quest hand tracking into ROS2 and Isaac Lab. The current hand-control mapping is an early prototype. The next production-quality step is a proper retargeter:

```text
Quest landmarks
  -> palm-local coordinates
  -> normalized finger curl and thumb opposition
  -> smoothing and joint limits
  -> Wato hand joint targets
  -> wrist/arm IK
```

