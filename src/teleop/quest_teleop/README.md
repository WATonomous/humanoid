# quest_teleop

Quest 2 hand tracking → Pioneer bimanual arms.

```
Quest browser (WebXR) ──WSS──▶ bridge/ ──/quest_teleop (ROS 2)──▶ sim/ ──▶ Isaac Sim arms
```

| Folder | ROS 2 package | What it does |
|---|---|---|
| [`bridge/`](bridge/) | `quest_teleop` (C++) | Serves the WebXR page, receives hand poses over WSS, publishes `/quest_teleop` |
| [`sim/`](sim/) | `quest_isaac_teleop` (Python) | Subscribes to `/quest_teleop`, IK for both arms in Isaac Sim, recording, headset view |

Certificates (`cert.pem`, `key.pem`) live in `certs/` here, mounted into the container at `/certs`.

Full setup (certs, adb, launch): [`sim/README.md`](sim/README.md).
