import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python.vision import HandLandmarker, HandLandmarkerOptions, RunningMode
import urllib.request
import os
import math
import roslibpy
import json

# ── Model download ────────────────────────────────────────────────
model_path = "hand_landmarker.task"
if not os.path.exists(model_path):
    print("Downloading hand landmarker model...")
    urllib.request.urlretrieve(
        "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task",
        model_path
    )
    print("Model downloaded!")

# ── Finger indices ────────────────────────────────────────────────
FINGER_TIPS  = [4,  8,  12, 16, 20]
FINGER_MCPS  = [2,  5,  9,  13, 17]
FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]

def distance(a, b):
    return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)

def is_fist(hand_landmarks, threshold=0.07):
    curled = []
    for i in range(1, 5):
        tip = hand_landmarks[FINGER_TIPS[i]]
        mcp = hand_landmarks[FINGER_MCPS[i]]
        curled.append(distance(tip, mcp) < threshold)
    return all(curled), curled

# ── Connect to rosbridge ──────────────────────────────────────────
print("Connecting to rosbridge on localhost:9090 ...")
client = roslibpy.Ros(host="127.0.0.1", port=9090, is_secure=False)
client.run(timeout=10)
print("Connected!" if client.is_connected else "FAILED to connect")

# Two topics — one for landmarks, one for fist state
landmark_pub = roslibpy.Topic(client, '/wato/hand_landmarks', 'std_msgs/String')
fist_pub     = roslibpy.Topic(client, '/wato/fist_state',     'std_msgs/String')

# ── MediaPipe setup ───────────────────────────────────────────────
options = HandLandmarkerOptions(
    base_options=python.BaseOptions(model_asset_path=model_path),
    running_mode=RunningMode.IMAGE,
    num_hands=1,
    min_hand_detection_confidence=0.7,
    min_hand_presence_confidence=0.7,
    min_tracking_confidence=0.5
)

cap = cv2.VideoCapture(0)
print("Streaming hand data to container. Press q to quit.")

with HandLandmarker.create_from_options(options) as landmarker:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        rgb    = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = landmarker.detect(mp_img)

        if result.hand_landmarks:
            for hand in result.hand_landmarks:
                h, w, _ = frame.shape

                # Draw landmarks
                for landmark in hand:
                    cx, cy = int(landmark.x * w), int(landmark.y * h)
                    cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

                # Fist detection
                fist, curled = is_fist(hand)

                # Build landmark payload
                landmarks_data = [
                    {"x": lm.x, "y": lm.y, "z": lm.z}
                    for lm in hand
                ]

                # Publish landmarks to container
                landmark_pub.publish(roslibpy.Message({
                    "data": json.dumps(landmarks_data)
                }))

                # Publish fist state to container
                fist_pub.publish(roslibpy.Message({
                    "data": json.dumps({
                        "fist": fist,
                        "fingers": {
                            FINGER_NAMES[i+1]: curled[i]
                            for i in range(4)
                        }
                    })
                }))

                # Draw status on screen
                for i, (name, curl) in enumerate(zip(FINGER_NAMES[1:], curled)):
                    color  = (0, 0, 255) if curl else (0, 255, 0)
                    cv2.putText(frame, f"{name}: {'CURLED' if curl else 'open'}",
                                (10, 30 + i*25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                label = "FIST CLOSED" if fist else "hand open"
                color = (0, 0, 255)   if fist else (0, 255, 0)
                cv2.putText(frame, label, (10, 200),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)

        cv2.imshow("Hand Detection - Sending to Isaac Lab", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
client.terminate()
print("Done.")