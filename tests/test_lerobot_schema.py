from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from pico_isaaclab.joint_contract import CAMERA_FEATURE_BY_NAME, CAMERA_NAMES, JOINT_NAMES_23
from pico_isaaclab.lerobot_recorder import LeRobotRecorder, RecorderConfig


class LeRobotSchemaTest(unittest.TestCase):
    def test_stable_v2_v3_feature_contract(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            recorder = LeRobotRecorder(
                RecorderConfig(
                    root=Path(directory) / "dataset",
                    repo_id="local/test",
                    fps=15,
                    width=640,
                    height=360,
                )
            )
            try:
                features = recorder._features()
                self.assertEqual(features["observation.state"]["shape"], (23,))
                self.assertEqual(features["action"]["names"], {"motors": list(JOINT_NAMES_23)})
                for name in CAMERA_NAMES:
                    self.assertEqual(features[CAMERA_FEATURE_BY_NAME[name]]["shape"], (360, 640, 3))
            finally:
                recorder.close()


if __name__ == "__main__":
    unittest.main()

