"""Import this from a scene script to expose several camera angles to the
isaac_multiview_screenshot.sh harness in one running process (no relaunching
between views, so physics state stays consistent across shots).

Usage in your scene script, after building the scene and calling sim.reset():

    from multiview_capture import capture_views
    capture_views(sim, simulation_app, [
        ("front", (1.5, -1.6, 1.4), (0.6, -0.2, 0.1)),
        ("top",   (0.6, -0.2, 2.2), (0.6, -0.2, 0.0)),
        ("side",  (0.6, -2.0, 0.6), (0.6, -0.2, 0.1)),
    ])

For each (name, eye, target): sets the camera, lets a few frames render,
prints "VIEW_READY:<name>" (the harness watches for this), then blocks until
the harness signals it has captured that view before moving to the next one —
so views can't be captured mid-transition or race each other.
"""
import os
import time


def capture_views_headless(sim, simulation_app, views, out_dir="/tmp/isaac_multiview_out"):
    """Headless variant: renders each view straight to a PNG via Isaac Sim's
    own viewport capture — no window, no X11, no xwd round-trip. Much faster
    than the GUI+xwd path (see capture_views below) since there's no window
    to race against: each capture is written to disk before we move on, so no
    advance-signal handshake with the harness is needed. Call
    isaac_headless_multiview.sh to drive this from the host.
    """
    # NOTE: omni.kit.viewport.utility.capture_viewport_to_file silently no-ops
    # in headless mode — get_active_viewport() needs an actual GUI viewport,
    # which doesn't exist headless even with --enable_cameras. The real
    # headless render path in IsaacLab is a Camera sensor reading
    # .data.output["rgb"] (same as pick_place_bimanual's cameras) and saving
    # that tensor with PIL.
    import numpy as np
    import torch
    from PIL import Image

    import isaaclab.sim as sim_utils
    from isaaclab.sensors import Camera, CameraCfg

    os.makedirs(out_dir, exist_ok=True)

    cam_cfg = CameraCfg(
        prim_path="/World/MultiviewCaptureCamera",
        update_period=0,
        height=720,
        width=1280,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(focal_length=18.0, clipping_range=(0.05, 20.0)),
    )
    camera = Camera(cfg=cam_cfg)
    # sensors created after the initial sim.reset() need a second reset to
    # initialize their internal buffers (e.g. _ALL_INDICES) — AttributeError
    # otherwise on first use.
    sim.reset()

    for _ in range(5):
        sim.step()
        simulation_app.update()

    for name, eye, target in views:
        eyes = torch.tensor([list(eye)], dtype=torch.float32, device=camera.device)
        targets = torch.tensor([list(target)], dtype=torch.float32, device=camera.device)
        camera.set_world_poses_from_view(eyes, targets)

        for _ in range(10):
            sim.step()
            simulation_app.update()
        camera.update(dt=0.0, force_recompute=True)

        rgb = camera.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8)
        path = os.path.join(out_dir, f"{name}.png")
        Image.fromarray(rgb).save(path)
        print(f"VIEW_SAVED:{name}", flush=True)

    print("ALL_VIEWS_SAVED", flush=True)


def capture_views(sim, simulation_app, views, signal_dir="/tmp/isaac_multiview_signals"):
    os.makedirs(signal_dir, exist_ok=True)
    for name, eye, target in views:
        sim.set_camera_view(list(eye), list(target))
        for _ in range(10):
            sim.step()
            simulation_app.update()

        print(f"VIEW_READY:{name}", flush=True)

        signal_path = os.path.join(signal_dir, f"{name}.advance")
        while not os.path.exists(signal_path):
            simulation_app.update()
            time.sleep(0.05)
        os.remove(signal_path)
