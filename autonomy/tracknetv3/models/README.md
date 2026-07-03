# TrackNetV3 Models

Place local TrackNetV3 checkpoints in `models/weights/` or mount them into the
container at `/models/tracknetv3`.

Only checkpoint files are required for runtime inference. Do not vendor the full
upstream TrackNetV3 repository into this directory unless training or upstream
evaluation tooling becomes part of the task.

Expected filenames:

- `TrackNet_best.pt`
- `InpaintNet_best.pt` optional and experimental for the ROS wrapper

Optional checksum files may be placed next to checkpoints using standard
`sha256sum` format:

```bash
sha256sum TrackNet_best.pt > TrackNet_best.pt.sha256
```

Checkpoint files are intentionally ignored by git.
