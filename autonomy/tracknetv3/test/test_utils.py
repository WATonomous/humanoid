import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from tracknetv3.utils import (
    FrameSequenceBuffer,
    format_stats_json,
    scale_point_to_image,
    verify_sha256_file,
)


class TestTrackNetV3Utils(unittest.TestCase):

    def test_frame_sequence_buffer_returns_latest_complete_window(self):
        buffer = FrameSequenceBuffer(seq_len=3)

        self.assertIsNone(buffer.append("frame-0"))
        self.assertIsNone(buffer.append("frame-1"))
        self.assertEqual(buffer.append("frame-2"), ["frame-0", "frame-1", "frame-2"])
        self.assertEqual(buffer.append("frame-3"), ["frame-1", "frame-2", "frame-3"])

    def test_scale_point_to_image_maps_model_coordinates(self):
        self.assertEqual(
            scale_point_to_image(
                x=256,
                y=144,
                model_width=512,
                model_height=288,
                image_width=1280,
                image_height=720,
            ),
            (640, 360),
        )

    def test_format_stats_json_is_compact_and_sorted(self):
        stats = format_stats_json(
            active=True,
            device="cpu",
            fps=29.987,
            inference_ms=12.3456,
            visible=False,
        )

        self.assertEqual(
            json.loads(stats),
            {
                "active": True,
                "device": "cpu",
                "fps": 29.99,
                "inference_ms": 12.35,
                "visible": False,
            },
        )
        self.assertNotIn(" ", stats)

    def test_verify_sha256_file_accepts_standard_checksum_file(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            model_path = Path(tmp_dir) / "TrackNet_best.pt"
            model_path.write_bytes(b"checkpoint")
            checksum = hashlib.sha256(b"checkpoint").hexdigest()

            checksum_path = Path(tmp_dir) / "TrackNet_best.pt.sha256"
            checksum_path.write_text(f"{checksum}  TrackNet_best.pt\n", encoding="utf-8")

            self.assertTrue(verify_sha256_file(model_path, checksum_path))

    def test_verify_sha256_file_rejects_mismatched_checksum(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            model_path = Path(tmp_dir) / "TrackNet_best.pt"
            model_path.write_bytes(b"checkpoint")

            checksum_path = Path(tmp_dir) / "TrackNet_best.pt.sha256"
            checksum_path.write_text(f"{'0' * 64}  TrackNet_best.pt\n", encoding="utf-8")

            self.assertFalse(verify_sha256_file(model_path, checksum_path))


if __name__ == "__main__":
    unittest.main()
