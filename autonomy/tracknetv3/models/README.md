# TrackNetV3 Models

Place local TrackNetV3 checkpoints in `models/weights/` or mount them into the
container at `/models/tracknetv3`.

Expected filenames:

- `TrackNet_best.pt`
- `InpaintNet_best.pt` optional

Optional checksum files may be placed next to checkpoints using standard
`sha256sum` format:

```bash
sha256sum TrackNet_best.pt > TrackNet_best.pt.sha256
```

Checkpoint files are intentionally ignored by git.
