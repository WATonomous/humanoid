import unittest
from importlib.util import find_spec

if find_spec("numpy") is None or find_spec("PIL") is None:
    raise unittest.SkipTest("TrackNetV3 inference tests require numpy and Pillow")

from helpers import install_inference_stubs

install_inference_stubs()

import numpy as np

from tracknetv3.inference import TrackNetV3Inference, heatmap_to_point


class TestTrackNetV3Inference(unittest.TestCase):

    def test_heatmap_to_point_returns_center_of_confident_region(self):
        heatmap = np.zeros((288, 512), dtype=np.float32)
        heatmap[40:44, 100:104] = 0.9

        self.assertEqual(heatmap_to_point(heatmap, 0.5), (102, 42, True))

    def test_heatmap_to_point_hides_low_confidence_output(self):
        heatmap = np.zeros((288, 512), dtype=np.float32)
        heatmap[40:44, 100:104] = 0.49

        self.assertEqual(heatmap_to_point(heatmap, 0.5), (0, 0, False))

    def test_subtract_preprocessing_clips_background_difference(self):
        inference = object.__new__(TrackNetV3Inference)
        inference.bg_mode = "subtract"
        frames = [
            np.zeros((8, 8, 3), dtype=np.uint8),
            np.full((8, 8, 3), 255, dtype=np.uint8),
        ]

        model_input = inference._preprocess_sequence(frames)

        self.assertEqual(model_input.shape, (1, 2, 288, 512))
        self.assertEqual(float(model_input.max()), 1.0)


if __name__ == "__main__":
    unittest.main()
