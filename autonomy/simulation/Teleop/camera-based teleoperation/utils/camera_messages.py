"""
camera_messages.py
==================
This script streams live webcam video to the Isaac Lab Docker container
over a rosbridge WebSocket connection.

Process:
  1. Connect to rosbridge on localhost:9090 (via SSH tunnel from Docker)
  2. Capture frames from the local webcam using OpenCV
  3. JPEG-compress and base64-encode each frame
  4. Publish each encoded frame to the ROS2 topic
     /camera/image_raw_compressed so the simulation can receive video
"""
import cv2

import roslibpy
import base64
import json

# Replace with your server's IP
SERVER_IP = "localhost"
SERVER_PORT = 9090

client = roslibpy.Ros(host=SERVER_IP, port=SERVER_PORT)
client.run()

publisher = roslibpy.Topic(
    client,
    '/camera/image_raw_compressed',
    'sensor_msgs/CompressedImage'
)

cap = cv2.VideoCapture(0)  # your laptop webcam
print("Streaming webcam to Isaac Lab server... Press Ctrl+C to stop.")

try:
    while client.is_connected:
        ret, frame = cap.read()
        if not ret:
            continue

        # Compress frame to JPEG
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        encoded = base64.b64encode(buffer).decode('utf-8')

        msg = roslibpy.Message({
            'format': 'jpeg',
            'data': encoded
        })
        publisher.publish(msg)

        # Show local preview
        cv2.imshow("Sending to Isaac Lab", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass

cap.release()
cv2.destroyAllWindows()
client.terminate()