from pathlib import Path
import unittest


class TestTrackNetV3LaunchConfig(unittest.TestCase):

    def test_launch_file_exposes_runtime_override_arguments(self):
        launch_path = Path(__file__).parents[1] / "launch" / "tracknetv3.launch.py"
        launch_text = launch_path.read_text(encoding="utf-8")

        for argument in (
            "model_dir",
            "tracknet_checkpoint",
            "inpaintnet_checkpoint",
            "device",
            "confidence_threshold",
            "enable_debug",
            "log_level",
        ):
            self.assertIn(f'"{argument}"', launch_text)

    def test_default_config_keeps_public_pixel_detection_topics(self):
        config_path = Path(__file__).parents[1] / "config" / "tracknetv3_config.yaml"
        config_text = config_path.read_text(encoding="utf-8")

        self.assertIn('rgb_image_topic: "/camera/color/image_raw"', config_text)
        self.assertIn('point_topic: "/tracknetv3/shuttle_point"', config_text)
        self.assertIn('visible_topic: "/tracknetv3/shuttle_visible"', config_text)
        self.assertIn('debug_image_topic: "/tracknetv3/debug_image"', config_text)
        self.assertIn('stats_topic: "/tracknetv3/stats"', config_text)


if __name__ == "__main__":
    unittest.main()
