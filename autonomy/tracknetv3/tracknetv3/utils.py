import hashlib
import json
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple


class FrameSequenceBuffer:
    """Fixed-size sliding frame buffer for sequence inference."""

    def __init__(self, seq_len: int):
        if seq_len <= 0:
            raise ValueError("seq_len must be positive")
        self.seq_len = seq_len
        self._frames: Deque[Any] = deque(maxlen=seq_len)

    def append(self, frame: Any) -> Optional[List[Any]]:
        """Append a frame and return the latest complete window if available."""
        self._frames.append(frame)
        if len(self._frames) < self.seq_len:
            return None
        return list(self._frames)

    def clear(self) -> None:
        """Clear all buffered frames."""
        self._frames.clear()

    def __len__(self) -> int:
        return len(self._frames)


def scale_point_to_image(
    x: float,
    y: float,
    model_width: int,
    model_height: int,
    image_width: int,
    image_height: int,
) -> Tuple[int, int]:
    """Scale a model-space point to the original image dimensions."""
    if model_width <= 0 or model_height <= 0:
        raise ValueError("model dimensions must be positive")
    if image_width <= 0 or image_height <= 0:
        raise ValueError("image dimensions must be positive")

    scaled_x = int(round(float(x) * image_width / model_width))
    scaled_y = int(round(float(y) * image_height / model_height))
    return scaled_x, scaled_y


def format_stats_json(**stats: Any) -> str:
    """Return compact, stable JSON for the stats topic."""
    rounded: Dict[str, Any] = {}
    for key, value in stats.items():
        if isinstance(value, float):
            rounded[key] = round(value, 2)
        else:
            rounded[key] = value
    return json.dumps(rounded, sort_keys=True, separators=(",", ":"))


def verify_sha256_file(model_path: Path, checksum_path: Path) -> bool:
    """Validate a model file against a standard sha256sum checksum file."""
    model_path = Path(model_path)
    checksum_path = Path(checksum_path)

    if not model_path.is_file() or not checksum_path.is_file():
        return False

    line = checksum_path.read_text(encoding="utf-8").strip().splitlines()[0]
    expected = line.split()[0].lower()
    if len(expected) != 64:
        return False

    digest = hashlib.sha256()
    with model_path.open("rb") as model_file:
        for chunk in iter(lambda: model_file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().lower() == expected
