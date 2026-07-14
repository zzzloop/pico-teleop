"""Versioned transport contract shared by the ROS 2 bridge and simulator.

The wire format intentionally uses only JSON-compatible primitives so the
Isaac Lab Python environment does not need to share ROS 2's Python ABI.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Iterable


PROTOCOL_VERSION = 1
ALLOWED_COMMANDS = {
    "start",
    "pause",
    "resume",
    "stop",
    "reset",
    "calibrate",
    "record_start",
    "record_stop",
    "record_abort",
    "record_finalize",
}


def _finite_vector(values: Iterable[Any], size: int, label: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != size:
        raise ValueError(f"{label} must contain {size} values, got {len(result)}")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} contains NaN or Inf")
    return result


@dataclass(frozen=True)
class PoseSample:
    position: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    tracking: bool = True

    @classmethod
    def from_dict(cls, value: dict[str, Any], label: str) -> "PoseSample":
        position = _finite_vector(value.get("position", ()), 3, f"{label}.position")
        quaternion = _finite_vector(value.get("quaternion", ()), 4, f"{label}.quaternion")
        norm = math.sqrt(sum(component * component for component in quaternion))
        if norm < 1e-8:
            raise ValueError(f"{label}.quaternion is degenerate")
        normalized = tuple(component / norm for component in quaternion)
        return cls(position=position, quaternion_xyzw=normalized, tracking=bool(value.get("tracking", True)))

    def as_dict(self) -> dict[str, Any]:
        return {
            "position": list(self.position),
            "quaternion": list(self.quaternion_xyzw),
            "tracking": self.tracking,
        }


@dataclass(frozen=True)
class TeleopEvent:
    command: str
    task: str = ""
    event_id: int = 0

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "TeleopEvent":
        command = str(value.get("command", "")).strip().lower()
        if command not in ALLOWED_COMMANDS:
            raise ValueError(f"Unsupported teleoperation command: {command!r}")
        event_id = int(value.get("event_id", 0))
        if event_id < 0:
            raise ValueError("event_id must be non-negative")
        return cls(command=command, task=str(value.get("task", "")).strip(), event_id=event_id)

    def as_dict(self) -> dict[str, Any]:
        result = {"command": self.command}
        if self.task:
            result["task"] = self.task
        if self.event_id:
            result["event_id"] = self.event_id
        return result


@dataclass(frozen=True)
class TeleopPacket:
    sequence: int
    received_monotonic_ns: int
    sent_monotonic_ns: int
    left: PoseSample | None
    right: PoseSample | None
    head: PoseSample | None
    axes: tuple[float, ...]
    buttons: tuple[int, ...]
    events: tuple[TeleopEvent, ...]

    @classmethod
    def from_dict(
        cls,
        value: dict[str, Any],
        *,
        received_monotonic_ns: int | None = None,
    ) -> "TeleopPacket":
        protocol = int(value.get("protocol", -1))
        if protocol != PROTOCOL_VERSION:
            raise ValueError(f"Protocol version {protocol} is unsupported; expected {PROTOCOL_VERSION}")
        sequence = int(value.get("sequence", -1))
        if sequence < 0:
            raise ValueError("sequence must be non-negative")
        axes = tuple(float(item) for item in value.get("axes", ()))
        if not all(math.isfinite(item) for item in axes):
            raise ValueError("axes contains NaN or Inf")
        buttons = tuple(int(bool(item)) for item in value.get("buttons", ()))
        events = tuple(TeleopEvent.from_dict(item) for item in value.get("events", ()))

        def pose(name: str) -> PoseSample | None:
            raw = value.get(name)
            return None if raw is None else PoseSample.from_dict(raw, name)

        return cls(
            sequence=sequence,
            received_monotonic_ns=int(received_monotonic_ns or time.monotonic_ns()),
            sent_monotonic_ns=int(value.get("sent_monotonic_ns", 0)),
            left=pose("left"),
            right=pose("right"),
            head=pose("head"),
            axes=axes,
            buttons=buttons,
            events=events,
        )

    def as_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "type": "teleop",
            "protocol": PROTOCOL_VERSION,
            "sequence": self.sequence,
            "sent_monotonic_ns": self.sent_monotonic_ns,
            "axes": list(self.axes),
            "buttons": list(self.buttons),
            "events": [event.as_dict() for event in self.events],
        }
        for name in ("left", "right", "head"):
            sample = getattr(self, name)
            if sample is not None:
                result[name] = sample.as_dict()
        return result
