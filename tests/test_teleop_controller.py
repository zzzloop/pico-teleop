from __future__ import annotations

import time
import unittest

import numpy as np

from pico_isaaclab.protocol import PoseSample, TeleopEvent, TeleopPacket
from pico_isaaclab.teleop_controller import PicoTeleopController, SessionState, TeleopConfig, parse_axis_map


IDENTITY_ROT6D = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0]


def state() -> dict:
    left = [0.50, 0.20, 0.80, *IDENTITY_ROT6D, 0.0]
    right = [0.50, -0.20, 0.80, *IDENTITY_ROT6D, 0.0]
    return {"ready": True, "left_ee_base": left, "right_ee_base": right, "ee6d_base": left + right}


def packet(sequence: int, *, event: str | None = None, left_x: float = 0.0, age_s: float = 0.0) -> TeleopPacket:
    now_ns = time.monotonic_ns() - int(age_s * 1e9)
    events = () if event is None else (TeleopEvent(event, "pick the block", event_id=sequence),)
    return TeleopPacket(
        sequence=sequence,
        received_monotonic_ns=now_ns,
        sent_monotonic_ns=now_ns,
        left=PoseSample((left_x, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), True),
        right=PoseSample((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), True),
        head=None,
        axes=(0.5, 0.0, 1.0, 0.0),
        buttons=(),
        events=events,
    )


class TeleopControllerTest(unittest.TestCase):
    def test_start_calibrate_move_pause_resume_stop(self) -> None:
        controller = PicoTeleopController(TeleopConfig(orientation_mode="position_only"))
        actions = controller.ingest(packet(1, event="start"))
        self.assertEqual(controller.session_state, SessionState.RUNNING)
        self.assertEqual(actions[0].kind, "record_start")
        self.assertEqual(controller.advance(state()).reason, "calibrated_hold_one_frame")

        controller.ingest(packet(2, left_x=0.02))
        decision = controller.advance(state())
        self.assertEqual(decision.mode, "ee6d")
        self.assertAlmostEqual(decision.row[0], 0.52, places=6)
        self.assertAlmostEqual(decision.row[9], 0.0205, places=6)
        self.assertAlmostEqual(decision.row[19], 0.041, places=6)

        controller.ingest(packet(3, event="pause"))
        self.assertEqual(controller.advance(state()).mode, "hold")
        self.assertFalse(controller.recording_gate)
        controller.ingest(packet(4, event="resume", left_x=0.50))
        self.assertEqual(controller.advance(state()).reason, "calibrated_hold_one_frame")
        controller.ingest(packet(5, event="stop"))
        self.assertEqual(controller.session_state, SessionState.STOPPED)

    def test_watchdog_clears_calibration(self) -> None:
        controller = PicoTeleopController(TeleopConfig(command_timeout_s=0.05))
        controller.ingest(packet(1, event="start"))
        controller.advance(state())
        controller.ingest(packet(2, age_s=0.2))
        decision = controller.advance(state())
        self.assertEqual(decision.mode, "hold")
        self.assertTrue(decision.reason.startswith("watchdog_timeout"))
        self.assertIsNone(controller.calibration)

    def test_repeated_reliable_event_is_idempotent(self) -> None:
        controller = PicoTeleopController(TeleopConfig())
        first = packet(1, event="start")
        self.assertEqual(len(controller.ingest(first)), 1)
        repeated = TeleopPacket(
            sequence=2,
            received_monotonic_ns=time.monotonic_ns(),
            sent_monotonic_ns=0,
            left=first.left,
            right=first.right,
            head=None,
            axes=(),
            buttons=(),
            events=first.events,
        )
        self.assertEqual(controller.ingest(repeated), [])
        self.assertEqual(controller.status()["last_event_id"], 1)

    def test_axis_map_requires_right_handed_rotation(self) -> None:
        self.assertTrue(np.allclose(parse_axis_map("z,-x,-y") @ parse_axis_map("z,-x,-y").T, np.eye(3)))
        with self.assertRaises(ValueError):
            parse_axis_map("z,-x,y")


if __name__ == "__main__":
    unittest.main()
