

## One-sentence interview answer

“TLS is the encryption and identity layer used by HTTPS and secure WebSockets. In my Quest teleop project, it is required because WebXR hand tracking must run in a secure browser context, so the page is served over HTTPS and the hand data is streamed over WSS.”

# Summary: Quest WebXR Teleop Pipeline

## 1. What this code does

This pipeline turns Quest hand tracking into ROS2 data:

```text
Quest Browser
  -> WebXR hand tracking
  -> JSON over WSS
  -> C++ WebSocket server
  -> JSON parser
  -> ROS2 QuestHandPose message
  -> /quest_teleop topic
```

The code currently **streams hand data**. It does **not yet control the robot**.

The missing next layer is:

```text
/quest_teleop
  -> retarget Quest hand landmarks to robot joint targets
  -> clamp targets using URDF joint limits
  -> send commands to Isaac Sim or real robot
```

---

## 2. Python server summary

`webxr_server.py` starts an HTTPS server on port `8443`.

Its job is only to serve `index.html` to the Quest browser.

```text
https://<pc-ip>:8443
```

Key points:

```text
http.server -> serves static files
ssl -> enables HTTPS/TLS
STATIC_DIR -> folder containing index.html
CERT_DIR -> folder containing cert.pem and key.pem
0.0.0.0 -> listen on all network interfaces
serve_forever() -> keep server running
```

It does **not** parse hand data.

---

## 3. TLS summary

TLS is the security layer behind:

```text
HTTP  -> HTTPS
WS    -> WSS
```

Your project needs TLS because WebXR hand tracking usually requires a secure browser context.

So you use:

```text
https://<pc-ip>:8443
wss://<pc-ip>:9090
```

Certificate files:

```text
cert.pem -> public certificate
key.pem  -> private key
```

---

## 4. `index.html` summary

`index.html` runs inside the Quest browser.

When you click Start:

```text
1. Connect to C++ WSS server at wss://<pc-ip>:9090
2. Start WebXR immersive-vr session
3. Request hand tracking
4. Every XR frame, read hand joint poses
5. Build a JSON payload
6. Send payload over WebSocket
```

The key APIs are browser APIs, not Meta Quest imports:

```text
document
window
navigator.xr
WebSocket
XRWebGLLayer
```

No Meta Quest SDK import is needed because Quest Browser exposes WebXR directly.

---

## 5. What `jointSpace` and `jointPose` mean

```js
const jointSpace = hand.get(jointName);
const jointPose = jointSpace ? frame.getJointPose(jointSpace, refSpace) : null;
```

Meaning:

```text
jointSpace = reference/handle for a specific WebXR hand joint
jointPose  = actual position + orientation of that joint in this frame
```

`frame.getJointPose(jointSpace, refSpace)` asks:

```text
Where is this joint right now, relative to this coordinate frame?
```

Your code only pushes:

```js
jointPose.transform.position.x
jointPose.transform.position.y
jointPose.transform.position.z
```

So it uses only position, not orientation.

A clearer version would be:

```js
const jointPose = frame.getJointPose(jointSpace, refSpace);
const jointPosition = jointPose.transform.position;
```

---

## 6. Payload summary

Every XR frame, the browser creates a new payload:

```js
const payload = {
  left_wrist: emptyPose(),
  right_wrist: emptyPose(),
  left_hand_joints: Array(75).fill(0),
  right_hand_joints: Array(75).fill(0)
};
```

Then it fills in tracked hand data and sends it.

Why recreate every frame?

```text
Hand tracking changes every frame.
Fresh payload avoids stale hand data.
Missing hands stay as zeros/defaults.
C++ always receives the same message structure.
```

---

## 7. WebXR joints vs robot joints

The WebXR joint list is **not the same thing as robot joints**.

WebXR gives 3D landmarks:

```text
index-finger-metacarpal
index-finger-phalanx-proximal
index-finger-phalanx-intermediate
index-finger-phalanx-distal
index-finger-tip
```

Your robot has actuated joints like:

```text
mcp_index
pip_index
dip_index
```

So the mapping is:

```text
WebXR landmark positions
  -> compute bone vectors / angles
  -> estimate robot joint targets
  -> clamp using URDF limits
```

Do not treat WebXR landmark names as motor names.

---

## 8. Should the payload be named or array-based?

Current payload:

```json
"right_hand_joints": [
  0.1, 0.2, 0.3,
  0.4, 0.5, 0.6
]
```

This is compact but hard to debug.

Debug-friendly payload:

```json
"right_hand_joints": {
  "wrist": {"x": 0.1, "y": 0.2, "z": 0.3},
  "index-finger-tip": {"x": 0.4, "y": 0.5, "z": 0.6}
}
```

Recommendation:

```text
For debugging: named JSON object
For production: compact array with known joint_order
```

If you change this structure, the C++ parser must also change.

---

# Interview Questions and Answers

## Q1. What does your Quest teleop pipeline do?

It streams Quest hand-tracking data into ROS2. The Quest browser uses WebXR to read wrist poses and hand joint landmarks, sends them over a secure WebSocket as JSON, and a C++ ROS2 node parses that JSON into a typed message published on `/quest_teleop`.

---

## Q2. Does this code directly control the robot?

No. It only publishes raw hand-tracking data. Robot control needs another layer that subscribes to `/quest_teleop`, retargets the hand data to robot joint targets or end-effector targets, applies safety limits, and sends commands to Isaac Sim or the real robot.

---

## Q3. Why do you use HTTPS and WSS?

Because browser XR features like WebXR hand tracking require a secure context. HTTPS securely serves the WebXR page, and WSS securely streams live hand data from the Quest browser to the C++ server.

---

## Q4. What is TLS?

TLS is the encryption and identity layer behind HTTPS and WSS. It lets the browser create a secure connection to the server using a certificate and private key.

---

## Q5. What does the Python server do?

The Python server serves the static WebXR webpage over HTTPS on port `8443`. It does not receive or parse hand data. It only lets the Quest browser load `index.html` securely.

---

## Q6. What does `index.html` do?

It runs in the Quest browser. When the user clicks Start, it connects to the C++ WSS server, starts a WebXR VR session, requests hand tracking, reads hand joint poses every frame, builds a JSON payload, and sends it to the server.

---

## Q7. Why are there no Meta Quest imports?

Because the code uses standard browser APIs. Quest Browser exposes WebXR through `navigator.xr`, WebSocket through `WebSocket`, and webpage elements through `document`. No separate Meta Quest JavaScript library is needed.

---

## Q8. What is `navigator.xr`?

`navigator.xr` is the browser’s WebXR entry point. It lets JavaScript request an immersive VR session and access XR input sources like tracked hands.

---

## Q9. What is `jointSpace`?

`jointSpace` is a WebXR object representing the coordinate space of a specific hand joint. It is not the actual position. It is a reference to the joint’s tracked space.

---

## Q10. What does `frame.getJointPose(jointSpace, refSpace)` do?

It returns the current pose of that hand joint for the current XR frame, expressed relative to the chosen reference space. The pose contains position and orientation.

---

## Q11. Why does the code call it `jointPose` if it only uses x/y/z?

Because WebXR returns a full pose, which includes position and orientation. The current implementation only extracts the position. A cleaner implementation could extract `jointPosition` from `jointPose.transform.position`.

---

## Q12. What is `refSpace`?

`refSpace` is the coordinate frame used for WebXR poses. The code uses `"local"`, meaning the positions are relative to the Quest’s local tracking origin, not the robot base frame.

---

## Q13. Why is the Quest local frame a problem for robotics?

Because the robot expects commands in its own base frame. Quest coordinates must be transformed, scaled, and anchored before they can become meaningful robot targets.

---

## Q14. What does the hidden canvas do?

The hidden canvas provides a WebGL surface required by the WebXR immersive session. It is not used to send hand data. It is just required setup for the XR render layer.

---

## Q15. What is `gl`?

`gl` is the WebGL rendering context created from the canvas. WebXR uses it to create an `XRWebGLLayer` for the immersive session.

---

## Q16. Why create a new payload every frame?

Because hand tracking changes every frame. A fresh payload prevents stale data. If a hand disappears, the default zeros remain instead of accidentally reusing old hand positions.

---

## Q17. Is recreating the payload every frame slow?

Not for this prototype. The payload is small: roughly 25 joints times 3 coordinates times 2 hands, plus wrist poses. The bigger concerns are latency, jitter, JSON parsing cost, and downstream robot control timing.

---

## Q18. Why is the current hand joint array length 75?

Because it stores 25 WebXR hand landmarks, and each landmark has 3 coordinates:

```text
25 * 3 = 75
```

---

## Q19. Are WebXR joints the same as robot joints?

No. WebXR joints are tracked hand landmarks in 3D space. Robot joints are actuated degrees of freedom. You need a retargeting layer to convert landmarks into robot joint angles.

---

## Q20. For a 21-DOF robot hand, should you shrink the WebXR joint list?

Not immediately. Keep the full hand landmarks while developing retargeting. The extra landmarks help compute finger bend angles. Later, once the mapping is stable, you can optimize the payload.

---

## Q21. Why might a named JSON payload be better during development?

Because it is easier to debug. Instead of remembering that array index `36` belongs to a specific joint coordinate, you can inspect fields like `right_hand_joints["index-finger-tip"].x`.

---

## Q22. Why might an array payload be better in production?

Arrays are smaller and faster to parse. Once the joint order is fixed and documented, array-based messages reduce bandwidth and overhead.

---

## Q23. What is the safest architecture for the next step?

Keep the browser simple and add a retargeting node:

```text
index.html
  -> raw WebXR landmarks
C++ ROS2 bridge
  -> /quest_teleop
retargeting node
  -> robot joint targets
robot controller
  -> Isaac Sim or real robot
```

---

## Q24. What would you add before controlling real hardware?

I would add timestamps, deadman control, workspace limits, joint limit clamping, low-pass filtering, message timeout detection, and emergency stop behavior.

---

## Q25. What is the main technical challenge after streaming hand data?

The main challenge is retargeting. The system must convert Quest hand landmarks from the Quest local frame into robot joint commands while handling different kinematics, joint limits, scaling, latency, and safety constraints.
