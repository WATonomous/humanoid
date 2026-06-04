"""
websocket_test.py
=================
Minimal connectivity test.
Opens a raw WebSocket connection to the rosbridge server on port 9090
and sends one "advertise" command to confirm the network path between
the Windows PC and the Docker container is working.
Run this before starting hand_recorder.py to verify the SSH tunnel is up.
"""
import asyncio

import websockets

async def test():
    try:
        async with websockets.connect("ws://127.0.0.1:9090") as ws:
            print("WebSocket connected successfully!")
            await ws.send('{"op":"advertise","topic":"/test","type":"std_msgs/String"}')
            print("Message sent!")
    except Exception as e:
        print(f"Failed: {e}")

asyncio.run(test())