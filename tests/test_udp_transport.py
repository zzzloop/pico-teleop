from __future__ import annotations

import json
import socket
import time
import unittest

from pico_isaaclab.protocol import PROTOCOL_VERSION
from pico_isaaclab.transports import UdpTeleopTransport


class UdpTransportTest(unittest.TestCase):
    def test_packet_and_status_round_trip(self) -> None:
        transport = UdpTeleopTransport("127.0.0.1", 0, status_hz=1000.0)
        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client.settimeout(1.0)
        try:
            port = transport.socket.getsockname()[1]
            client.sendto(
                json.dumps(
                    {
                        "type": "teleop",
                        "protocol": PROTOCOL_VERSION,
                        "sequence": 1,
                        "left": {"position": [0, 0, 0], "quaternion": [0, 0, 0, 1]},
                        "right": {"position": [0, 0, 0], "quaternion": [0, 0, 0, 1]},
                        "events": [{"command": "pause", "event_id": 123}],
                    }
                ).encode("utf-8"),
                ("127.0.0.1", port),
            )
            packets = []
            deadline = time.monotonic() + 1.0
            while not packets and time.monotonic() < deadline:
                packets = transport.poll()
            self.assertEqual(packets[0].events[0].event_id, 123)
            transport.publish_status({"teleop": {"last_event_id": 123}})
            response, _ = client.recvfrom(65535)
            status = json.loads(response.decode("utf-8"))
            self.assertEqual(status["status"]["teleop"]["last_event_id"], 123)
        finally:
            client.close()
            transport.close()


if __name__ == "__main__":
    unittest.main()

