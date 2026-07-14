from __future__ import annotations

import unittest
from pathlib import Path

from pico_isaaclab.joint_contract import JOINT_NAMES_23, inspect_urdf_contract, require_vector23


class JointContractTest(unittest.TestCase):
    def test_public_vector(self) -> None:
        self.assertEqual(len(JOINT_NAMES_23), 23)
        self.assertEqual(require_vector23([0] * 23, "test"), [0.0] * 23)
        with self.assertRaises(ValueError):
            require_vector23([0] * 22, "test")

    def test_reference_urdf_when_available(self) -> None:
        path = Path("D:/IsaacLabCode/BRX042501/BRX042501_wheel_4cams.urdf")
        if not path.is_file():
            self.skipTest("reference IsaacLabCode project is not available")
        report = inspect_urdf_contract(path)
        self.assertEqual(report.non_fixed_joint_count, 33)
        self.assertAlmostEqual(report.head_stereo_baseline_m, 0.060, places=6)


if __name__ == "__main__":
    unittest.main()

