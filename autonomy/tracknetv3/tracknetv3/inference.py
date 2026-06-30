import math
import time
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import torch
from PIL import Image

from tracknetv3.model import InpaintNet, create_tracknet
from tracknetv3.utils import scale_point_to_image, verify_sha256_file

MODEL_HEIGHT = 288
MODEL_WIDTH = 512


@dataclass(frozen=True)
class TrackNetV3Prediction:
    """Single-frame image-plane TrackNetV3 result."""

    x: int
    y: int
    visible: bool
    confidence: float
    inference_ms: float
    source: str


class TrackNetV3Inference:
    """Online TrackNetV3 inference wrapper for ROS image streams."""

    def __init__(
        self,
        tracknet_checkpoint: Path,
        inpaintnet_checkpoint: Optional[Path] = None,
        device: str = "cuda:0",
        use_fp16: bool = False,
        seq_len: int = 8,
        bg_mode: str = "concat",
        confidence_threshold: float = 0.5,
        history_length: int = 64,
    ):
        if seq_len <= 0:
            raise ValueError("seq_len must be positive")
        if bg_mode not in {"", "concat", "subtract", "subtract_concat"}:
            raise ValueError(f"Unsupported TrackNetV3 bg_mode: {bg_mode}")

        self.device = self._select_device(device)
        self.use_fp16 = bool(use_fp16 and self.device.type == "cuda")
        self.seq_len = seq_len
        self.bg_mode = bg_mode
        self.confidence_threshold = confidence_threshold
        self.history: Deque[TrackNetV3Prediction] = deque(maxlen=history_length)
        self._last_image_shape: Optional[Tuple[int, int]] = None

        self.tracknet, checkpoint_params = self._load_tracknet(tracknet_checkpoint)
        self.seq_len = int(checkpoint_params.get("seq_len", self.seq_len))
        self.bg_mode = str(checkpoint_params.get("bg_mode", self.bg_mode))
        self.tracknet.eval()

        self.inpaintnet = None
        self.inpaint_seq_len = 16
        if inpaintnet_checkpoint is not None:
            self.inpaintnet, inpaint_params = self._load_inpaintnet(inpaintnet_checkpoint)
            self.inpaint_seq_len = int(inpaint_params.get("seq_len", self.inpaint_seq_len))
            self.inpaintnet.eval()

    def predict(self, frames_bgr: Sequence[np.ndarray]) -> TrackNetV3Prediction:
        """Run TrackNetV3 on a complete frame sequence and return the newest point."""
        if len(frames_bgr) != self.seq_len:
            raise ValueError(f"expected {self.seq_len} frames, got {len(frames_bgr)}")

        start = time.perf_counter()
        image_height, image_width = frames_bgr[-1].shape[:2]
        self._last_image_shape = (image_width, image_height)

        model_input = self._preprocess_sequence(frames_bgr)
        tensor = torch.from_numpy(model_input).to(self.device)
        if self.use_fp16:
            tensor = tensor.half()

        with torch.no_grad():
            output = self.tracknet(tensor)

        heatmap = output[0, -1].detach().float().cpu().numpy()
        confidence = float(np.max(heatmap))
        model_x, model_y, visible = heatmap_to_point(heatmap, self.confidence_threshold)
        if visible:
            x, y = scale_point_to_image(
                model_x,
                model_y,
                MODEL_WIDTH,
                MODEL_HEIGHT,
                image_width,
                image_height,
            )
        else:
            x, y = 0, 0

        inference_ms = (time.perf_counter() - start) * 1000.0
        prediction = TrackNetV3Prediction(
            x=x,
            y=y,
            visible=visible,
            confidence=confidence,
            inference_ms=inference_ms,
            source="tracknet",
        )
        prediction = self._maybe_rectify_with_inpaintnet(prediction)
        self.history.append(prediction)
        return prediction

    def _load_tracknet(self, checkpoint_path: Path):
        checkpoint = self._load_checkpoint(checkpoint_path)
        params = checkpoint.get("param_dict", {})
        seq_len = int(params.get("seq_len", self.seq_len))
        bg_mode = str(params.get("bg_mode", self.bg_mode))
        if bg_mode not in {"", "concat", "subtract", "subtract_concat"}:
            raise ValueError(f"Unsupported TrackNetV3 bg_mode in checkpoint: {bg_mode}")
        model = create_tracknet(seq_len=seq_len, bg_mode=bg_mode)
        model.load_state_dict(checkpoint["model"])
        model = model.to(self.device)
        if self.use_fp16:
            model = model.half()
        return model, params

    def _load_inpaintnet(self, checkpoint_path: Path):
        checkpoint = self._load_checkpoint(checkpoint_path)
        model = InpaintNet()
        model.load_state_dict(checkpoint["model"])
        model = model.to(self.device)
        if self.use_fp16:
            model = model.half()
        return model, checkpoint.get("param_dict", {})

    def _load_checkpoint(self, checkpoint_path: Path) -> Dict:
        checkpoint_path = Path(checkpoint_path).expanduser()
        if not checkpoint_path.is_file():
            raise FileNotFoundError(f"TrackNetV3 checkpoint not found: {checkpoint_path}")

        checksum_path = checkpoint_path.with_name(checkpoint_path.name + ".sha256")
        if checksum_path.exists() and not verify_sha256_file(checkpoint_path, checksum_path):
            raise ValueError(f"Checksum validation failed for {checkpoint_path}")

        return torch.load(checkpoint_path, map_location=self.device)

    def _preprocess_sequence(self, frames_bgr: Sequence[np.ndarray]) -> np.ndarray:
        frames_rgb = [cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) for frame in frames_bgr]
        median_rgb = np.median(np.stack(frames_rgb, axis=0), axis=0).astype(np.uint8)

        channels: List[np.ndarray] = []
        if self.bg_mode == "concat":
            channels.extend(_resize_channels(median_rgb))

        for frame_rgb in frames_rgb:
            if self.bg_mode == "subtract":
                diff = _background_difference(frame_rgb, median_rgb)
                channels.append(_resize_gray(diff.astype(np.uint8)))
            elif self.bg_mode == "subtract_concat":
                channels.extend(_resize_channels(frame_rgb))
                diff = _background_difference(frame_rgb, median_rgb)
                channels.append(_resize_gray(diff.astype(np.uint8)))
            else:
                channels.extend(_resize_channels(frame_rgb))

        input_array = np.stack(channels, axis=0).astype(np.float32) / 255.0
        return input_array.reshape(1, input_array.shape[0], MODEL_HEIGHT, MODEL_WIDTH)

    def _maybe_rectify_with_inpaintnet(
        self,
        prediction: TrackNetV3Prediction,
    ) -> TrackNetV3Prediction:
        if self.inpaintnet is None or self._last_image_shape is None:
            return prediction

        recent = list(self.history)[-(self.inpaint_seq_len - 1):] + [prediction]
        if len(recent) < self.inpaint_seq_len:
            return prediction

        mask = generate_inpaint_mask(recent, th_h=self._last_image_shape[1] * 0.05)
        if mask[-1] != 1:
            return prediction

        image_width, image_height = self._last_image_shape
        coords = np.array([[item.x, item.y] for item in recent], dtype=np.float32)
        coords[:, 0] /= image_width
        coords[:, 1] /= image_height
        mask_array = np.array(mask, dtype=np.float32).reshape(1, self.inpaint_seq_len, 1)
        coord_tensor = torch.from_numpy(coords.reshape(1, self.inpaint_seq_len, 2)).to(self.device)
        mask_tensor = torch.from_numpy(mask_array).to(self.device)
        if self.use_fp16:
            coord_tensor = coord_tensor.half()
            mask_tensor = mask_tensor.half()

        with torch.no_grad():
            rectified = self.inpaintnet(coord_tensor, mask_tensor)
            rectified = rectified[0, -1].detach().float().cpu().numpy()

        x = int(round(float(rectified[0]) * image_width))
        y = int(round(float(rectified[1]) * image_height))
        return replace(prediction, x=x, y=y, visible=True, source="inpaintnet")

    @staticmethod
    def _select_device(requested: str) -> torch.device:
        if requested.startswith("cuda") and torch.cuda.is_available():
            return torch.device(requested)
        return torch.device("cpu")


def _resize_channels(image_rgb: np.ndarray) -> List[np.ndarray]:
    image = Image.fromarray(image_rgb.astype(np.uint8))
    resized = np.array(image.resize(size=(MODEL_WIDTH, MODEL_HEIGHT)))
    return [resized[:, :, channel] for channel in range(3)]


def _background_difference(frame_rgb: np.ndarray, median_rgb: np.ndarray) -> np.ndarray:
    return np.sum(
        np.abs(frame_rgb.astype(np.int16) - median_rgb.astype(np.int16)),
        axis=2,
    )


def _resize_gray(image: np.ndarray) -> np.ndarray:
    resized = Image.fromarray(image.astype(np.uint8)).resize(size=(MODEL_WIDTH, MODEL_HEIGHT))
    return np.array(resized)


def heatmap_to_point(
    heatmap: np.ndarray,
    confidence_threshold: float,
) -> Tuple[int, int, bool]:
    """Convert a TrackNet heatmap to a model-space point."""
    if float(np.max(heatmap)) < confidence_threshold:
        return 0, 0, False

    binary = (heatmap > confidence_threshold).astype(np.uint8) * 255
    contours, _ = cv2.findContours(binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0, 0, False

    x, y, width, height = max(
        (cv2.boundingRect(contour) for contour in contours),
        key=lambda rect: rect[2] * rect[3],
    )
    return int(x + width / 2), int(y + height / 2), True


def generate_inpaint_mask(
    predictions: Sequence[TrackNetV3Prediction],
    th_h: float,
) -> List[int]:
    """Generate the TrackNetV3 inpaint mask for recent predictions."""
    y = np.array([prediction.y for prediction in predictions])
    vis_pred = np.array([1 if prediction.visible else 0 for prediction in predictions])
    inpaint_mask = np.zeros_like(y)
    i = 0
    j = 0
    while j < len(vis_pred):
        while i < len(vis_pred) - 1 and vis_pred[i] == 1:
            i += 1
        j = i
        while j < len(vis_pred) - 1 and vis_pred[j] == 0:
            j += 1
        if j == i:
            break
        if i == 0 and y[j] > th_h:
            inpaint_mask[:j] = 1
        elif (i > 1 and y[i - 1] > th_h) and (j < len(vis_pred) and y[j] > th_h):
            inpaint_mask[i:j] = 1
        i = j
    return inpaint_mask.tolist()
