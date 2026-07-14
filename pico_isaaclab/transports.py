"""Server-local ROS 2 bridge transport for the Isaac Lab process."""

from __future__ import annotations

import json
import socket
import time
from typing import Any

from .protocol import PROTOCOL_VERSION, TeleopPacket


class UdpTeleopTransport:
    """Non-blocking adapter for the server-local ROS 2 bridge.

    The production topology deliberately binds this socket to loopback. PICO
    traffic reaches the server through Unity ROS-TCP; UDP is never exposed to
    the workstation network.
    """

    def __init__(self, host: str, port: int, status_hz: float = 10.0) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((host, int(port)))
        self.socket.setblocking(False)
        self.last_sender: tuple[str, int] | None = None
        self.last_error: str | None = None
        self.status_period_s = 1.0 / max(float(status_hz), 0.1)
        self.last_status_t = 0.0
        bound_host, bound_port = self.socket.getsockname()
        print(f"[PICO][UDP] listening on udp://{bound_host}:{bound_port}")

    def poll(self) -> list[TeleopPacket]:
        packets: list[TeleopPacket] = []
        while True:
            try:
                payload, sender = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            except OSError as exc:
                self.last_error = str(exc)
                break
            try:
                raw = json.loads(payload.decode("utf-8"))
                if raw.get("type", "teleop") != "teleop":
                    continue
                packets.append(TeleopPacket.from_dict(raw, received_monotonic_ns=time.monotonic_ns()))
                self.last_sender = sender
                self.last_error = None
            except Exception as exc:
                self.last_error = str(exc)
                print(f"[PICO][UDP] rejected packet from {sender}: {exc}")
        return packets

    def publish_status(self, status: dict[str, Any]) -> None:
        now = time.monotonic()
        if self.last_sender is None or now - self.last_status_t < self.status_period_s:
            return
        compact = {
            "type": "status",
            "protocol": PROTOCOL_VERSION,
            "sent_monotonic_ns": time.monotonic_ns(),
            "status": status,
        }
        try:
            self.socket.sendto(json.dumps(compact, separators=(",", ":")).encode("utf-8"), self.last_sender)
            self.last_status_t = now
        except OSError as exc:
            self.last_error = str(exc)

    def status(self) -> dict[str, Any]:
        return {
            "kind": "udp",
            "connected": self.last_sender is not None,
            "last_sender": None if self.last_sender is None else f"{self.last_sender[0]}:{self.last_sender[1]}",
            "last_error": self.last_error,
        }

    def close(self) -> None:
        self.socket.close()
