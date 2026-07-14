from __future__ import annotations

import unittest

from pico_isaaclab.protocol import PROTOCOL_VERSION, TeleopPacket


class ProtocolTest(unittest.TestCase):
    def test_round_trip(self) -> None:
        raw = {
            "type": "teleop",
            "protocol": PROTOCOL_VERSION,
            "sequence": 7,
            "sent_monotonic_ns": 123,
            "left": {"position": [1, 2, 3], "quaternion": [0, 0, 0, 2], "tracking": True},
            "right": {"position": [4, 5, 6], "quaternion": [0, 0, 0, 1], "tracking": True},
            "axes": [0.1, 0.2, 0.3, 0.4],
            "buttons": [1, 0, 0, 0, 0],
            "events": [{"command": "start", "task": "pick the block", "event_id": 9}],
        }
        packet = TeleopPacket.from_dict(raw, received_monotonic_ns=456)
        self.assertEqual(packet.sequence, 7)
        self.assertEqual(packet.left.quaternion_xyzw, (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(packet.events[0].command, "start")
        self.assertEqual(packet.events[0].event_id, 9)
        self.assertEqual(packet.as_dict()["protocol"], PROTOCOL_VERSION)

    def test_rejects_bad_protocol_and_nan(self) -> None:
        with self.assertRaises(ValueError):
            TeleopPacket.from_dict({"protocol": 99, "sequence": 0})
        with self.assertRaises(ValueError):
            TeleopPacket.from_dict(
                {"protocol": PROTOCOL_VERSION, "sequence": 0, "axes": [float("nan")]}
            )


if __name__ == "__main__":
    unittest.main()
