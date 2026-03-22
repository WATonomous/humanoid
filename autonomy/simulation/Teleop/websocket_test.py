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