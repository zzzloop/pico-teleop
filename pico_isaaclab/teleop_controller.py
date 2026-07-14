"""Safety state machine and calibrated PICO-controller to BRX EE mapping."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Literal

import numpy as np

from .protocol import PoseSample, TeleopEvent, TeleopPacket


class SessionState(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    STOPPED = "stopped"


@dataclass(frozen=True)
class TeleopConfig:
    scale: float = 1.0
    axis_map: str = "x,y,z"
    orientation_mode: Literal["full", "position_only"] = "full"
    command_timeout_s: float = 0.35
    max_step_m: float = 0.04
    max_rotation_step_deg: float = 12.0
    workspace_min: tuple[float, float, float] = (-0.20, -1.20, 0.35)
    workspace_max: tuple[float, float, float] = (1.20, 1.20, 1.35)
    left_trigger_axis: int = 0
    right_trigger_axis: int = 2
    gripper_open_m: float = 0.0
    gripper_closed_m: float = 0.041
    auto_record_on_start: bool = True
    default_task: str = "Teleoperate BRX042501 with PICO controllers."


@dataclass(frozen=True)
class ControllerAction:
    kind: Literal["record_start", "record_stop", "record_abort", "record_finalize", "reset"]
    task: str = ""


@dataclass(frozen=True)
class ControlDecision:
    mode: Literal["ee6d", "hold", "none"]
    row: tuple[float, ...] | None = None
    reason: str = ""


@dataclass(frozen=True)
class _Calibration:
    left_hand_position: np.ndarray
    right_hand_position: np.ndarray
    left_hand_rotation: np.ndarray
    right_hand_rotation: np.ndarray
    left_ee_position: np.ndarray
    right_ee_position: np.ndarray
    left_ee_rotation: np.ndarray
    right_ee_rotation: np.ndarray


def parse_axis_map(spec: str) -> np.ndarray:
    axes = {"x": 0, "y": 1, "z": 2}
    matrix = np.zeros((3, 3), dtype=np.float64)
    tokens = [token.strip().lower() for token in spec.split(",")]
    if len(tokens) != 3:
        raise ValueError("axis_map must contain exactly three comma-separated axes")
    used: set[int] = set()
    for row, token in enumerate(tokens):
        sign = -1.0 if token.startswith("-") else 1.0
        token = token[1:] if token.startswith(("-", "+")) else token
        if token not in axes:
            raise ValueError(f"Bad axis_map token: {token!r}")
        column = axes[token]
        if column in used:
            raise ValueError("axis_map must use x, y and z exactly once")
        used.add(column)
        matrix[row, column] = sign
    determinant = float(np.linalg.det(matrix))
    if not math.isclose(determinant, 1.0, abs_tol=1e-6):
        raise ValueError(
            f"axis_map must be a right-handed rotation (determinant +1), got {determinant:.1f}"
        )
    return matrix


def quaternion_xyzw_to_matrix(quaternion: tuple[float, ...] | np.ndarray) -> np.ndarray:
    x, y, z, w = np.asarray(quaternion, dtype=np.float64)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-8:
        raise ValueError("Quaternion is degenerate")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion_xyzw(matrix: np.ndarray) -> np.ndarray:
    m = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = np.asarray([(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s])
    else:
        diagonal = int(np.argmax(np.diag(m)))
        if diagonal == 0:
            s = math.sqrt(max(1.0 + m[0, 0] - m[1, 1] - m[2, 2], 0.0)) * 2.0
            q = np.asarray([0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s])
        elif diagonal == 1:
            s = math.sqrt(max(1.0 + m[1, 1] - m[0, 0] - m[2, 2], 0.0)) * 2.0
            q = np.asarray([(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s])
        else:
            s = math.sqrt(max(1.0 + m[2, 2] - m[0, 0] - m[1, 1], 0.0)) * 2.0
            q = np.asarray([(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s])
    return q / max(float(np.linalg.norm(q)), 1e-12)


def rot6d_to_matrix(values: Any) -> np.ndarray:
    raw = np.asarray(values, dtype=np.float64).reshape(6)
    first = raw[0:3]
    first /= max(float(np.linalg.norm(first)), 1e-12)
    second = raw[3:6] - np.dot(first, raw[3:6]) * first
    second /= max(float(np.linalg.norm(second)), 1e-12)
    third = np.cross(first, second)
    return np.stack((first, second, third), axis=1)


def matrix_to_rot6d(matrix: np.ndarray) -> np.ndarray:
    rotation = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    return np.concatenate((rotation[:, 0], rotation[:, 1]))


def _slerp_matrix(current: np.ndarray, target: np.ndarray, max_angle_rad: float) -> np.ndarray:
    current_q = matrix_to_quaternion_xyzw(current)
    target_q = matrix_to_quaternion_xyzw(target)
    dot = float(np.dot(current_q, target_q))
    if dot < 0.0:
        target_q = -target_q
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    angle = 2.0 * math.acos(dot)
    if angle <= max_angle_rad or angle < 1e-8:
        return target
    fraction = max_angle_rad / angle
    if dot > 0.9995:
        result = current_q + fraction * (target_q - current_q)
        result /= max(float(np.linalg.norm(result)), 1e-12)
    else:
        theta = math.acos(dot)
        sin_theta = math.sin(theta)
        result = (
            math.sin((1.0 - fraction) * theta) / sin_theta * current_q
            + math.sin(fraction * theta) / sin_theta * target_q
        )
    return quaternion_xyzw_to_matrix(result)


class PicoTeleopController:
    def __init__(self, cfg: TeleopConfig) -> None:
        self.cfg = cfg
        self.axis_rotation = parse_axis_map(cfg.axis_map)
        self.session_state = SessionState.IDLE
        self.latest_packet: TeleopPacket | None = None
        self.last_sequence = -1
        self.calibration: _Calibration | None = None
        self.safety_hold = True
        self.hold_reason = "not_started"
        self.task = cfg.default_task
        self.last_error: str | None = None
        self.last_event_id = 0
        self._processed_event_ids: set[int] = set()

    @property
    def recording_gate(self) -> bool:
        return self.session_state == SessionState.RUNNING and not self.safety_hold

    def ingest(self, packet: TeleopPacket) -> list[ControllerAction]:
        if packet.sequence <= self.last_sequence:
            return []
        self.last_sequence = packet.sequence
        self.latest_packet = packet
        actions: list[ControllerAction] = []
        for event in packet.events:
            if event.event_id and event.event_id in self._processed_event_ids:
                continue
            if event.event_id:
                self._processed_event_ids.add(event.event_id)
                self.last_event_id = max(self.last_event_id, event.event_id)
                if len(self._processed_event_ids) > 4096:
                    floor = max(0, self.last_event_id - 2048)
                    self._processed_event_ids = {
                        event_id for event_id in self._processed_event_ids if event_id >= floor
                    }
            actions.extend(self._handle_event(event))
        return actions

    def _handle_event(self, event: TeleopEvent) -> list[ControllerAction]:
        command = event.command
        if event.task:
            self.task = event.task
        if command == "start":
            self.session_state = SessionState.RUNNING
            self.calibration = None
            self.safety_hold = True
            self.hold_reason = "awaiting_calibration"
            return [ControllerAction("record_start", self.task)] if self.cfg.auto_record_on_start else []
        if command == "pause":
            self.session_state = SessionState.PAUSED
            self.safety_hold = True
            self.hold_reason = "paused"
            return []
        if command == "resume":
            self.session_state = SessionState.RUNNING
            self.calibration = None
            self.safety_hold = True
            self.hold_reason = "awaiting_recalibration"
            return []
        if command == "stop":
            self.session_state = SessionState.STOPPED
            self.calibration = None
            self.safety_hold = True
            self.hold_reason = "stopped"
            return [ControllerAction("record_stop")]
        if command == "reset":
            self.session_state = SessionState.IDLE
            self.calibration = None
            self.safety_hold = True
            self.hold_reason = "reset"
            return [ControllerAction("record_stop"), ControllerAction("reset")]
        if command == "calibrate":
            self.calibration = None
            self.safety_hold = True
            self.hold_reason = "awaiting_recalibration"
            return []
        if command in ("record_start", "record_stop", "record_abort", "record_finalize"):
            return [ControllerAction(command, event.task or self.task)]
        return []

    def _valid_packet(self, packet: TeleopPacket, now_ns: int) -> tuple[bool, str]:
        age_s = max(0.0, (now_ns - packet.received_monotonic_ns) / 1e9)
        if age_s > self.cfg.command_timeout_s:
            return False, f"watchdog_timeout:{age_s:.3f}s"
        if packet.left is None or packet.right is None:
            return False, "missing_controller_pose"
        if not packet.left.tracking or not packet.right.tracking:
            return False, "tracking_lost"
        return True, "ok"

    def _calibrate(self, packet: TeleopPacket, state: dict[str, Any]) -> None:
        assert packet.left is not None and packet.right is not None
        left_ee = np.asarray(state["left_ee_base"], dtype=np.float64)
        right_ee = np.asarray(state["right_ee_base"], dtype=np.float64)
        self.calibration = _Calibration(
            left_hand_position=np.asarray(packet.left.position, dtype=np.float64),
            right_hand_position=np.asarray(packet.right.position, dtype=np.float64),
            left_hand_rotation=quaternion_xyzw_to_matrix(packet.left.quaternion_xyzw),
            right_hand_rotation=quaternion_xyzw_to_matrix(packet.right.quaternion_xyzw),
            left_ee_position=left_ee[0:3].copy(),
            right_ee_position=right_ee[0:3].copy(),
            left_ee_rotation=rot6d_to_matrix(left_ee[3:9]),
            right_ee_rotation=rot6d_to_matrix(right_ee[3:9]),
        )

    def _target_pose(
        self,
        hand: PoseSample,
        hand_position_zero: np.ndarray,
        hand_rotation_zero: np.ndarray,
        ee_position_zero: np.ndarray,
        ee_rotation_zero: np.ndarray,
        current_ee: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        delta = np.asarray(hand.position, dtype=np.float64) - hand_position_zero
        target_position = ee_position_zero + self.cfg.scale * (self.axis_rotation @ delta)
        target_position = np.clip(target_position, self.cfg.workspace_min, self.cfg.workspace_max)
        current_position = current_ee[0:3]
        motion = target_position - current_position
        distance = float(np.linalg.norm(motion))
        if distance > self.cfg.max_step_m:
            target_position = current_position + motion / max(distance, 1e-12) * self.cfg.max_step_m

        current_rotation = rot6d_to_matrix(current_ee[3:9])
        if self.cfg.orientation_mode == "position_only":
            target_rotation = current_rotation
        else:
            hand_rotation = quaternion_xyzw_to_matrix(hand.quaternion_xyzw)
            source_delta = hand_rotation @ hand_rotation_zero.T
            mapped_delta = self.axis_rotation @ source_delta @ self.axis_rotation.T
            target_rotation = mapped_delta @ ee_rotation_zero
            target_rotation = _slerp_matrix(
                current_rotation,
                target_rotation,
                math.radians(self.cfg.max_rotation_step_deg),
            )
        return target_position, target_rotation

    def _axis(self, packet: TeleopPacket, index: int) -> float:
        if index < 0 or index >= len(packet.axes):
            return 0.0
        return float(np.clip(packet.axes[index], 0.0, 1.0))

    def _gripper(self, trigger: float) -> float:
        return self.cfg.gripper_open_m + trigger * (self.cfg.gripper_closed_m - self.cfg.gripper_open_m)

    def advance(self, state: dict[str, Any], now_ns: int | None = None) -> ControlDecision:
        now_ns = int(now_ns or time.monotonic_ns())
        if self.session_state != SessionState.RUNNING:
            self.safety_hold = True
            return ControlDecision("hold", reason=self.hold_reason)
        packet = self.latest_packet
        if packet is None:
            self.safety_hold = True
            self.hold_reason = "no_packet"
            return ControlDecision("hold", reason=self.hold_reason)
        valid, reason = self._valid_packet(packet, now_ns)
        if not valid:
            self.calibration = None
            self.safety_hold = True
            self.hold_reason = reason
            return ControlDecision("hold", reason=reason)
        if not state.get("ready") or "left_ee_base" not in state or "right_ee_base" not in state:
            self.safety_hold = True
            self.hold_reason = "sim_not_ready"
            return ControlDecision("hold", reason=self.hold_reason)
        if self.calibration is None:
            self._calibrate(packet, state)
            self.safety_hold = True
            self.hold_reason = "calibrated_hold_one_frame"
            return ControlDecision("hold", reason=self.hold_reason)

        assert packet.left is not None and packet.right is not None and self.calibration is not None
        left_current = np.asarray(state["left_ee_base"], dtype=np.float64)
        right_current = np.asarray(state["right_ee_base"], dtype=np.float64)
        left_position, left_rotation = self._target_pose(
            packet.left,
            self.calibration.left_hand_position,
            self.calibration.left_hand_rotation,
            self.calibration.left_ee_position,
            self.calibration.left_ee_rotation,
            left_current,
        )
        right_position, right_rotation = self._target_pose(
            packet.right,
            self.calibration.right_hand_position,
            self.calibration.right_hand_rotation,
            self.calibration.right_ee_position,
            self.calibration.right_ee_rotation,
            right_current,
        )
        row = np.concatenate(
            (
                left_position,
                matrix_to_rot6d(left_rotation),
                [self._gripper(self._axis(packet, self.cfg.left_trigger_axis))],
                right_position,
                matrix_to_rot6d(right_rotation),
                [self._gripper(self._axis(packet, self.cfg.right_trigger_axis))],
            )
        )
        if row.shape != (20,) or not np.all(np.isfinite(row)):
            self.safety_hold = True
            self.hold_reason = "invalid_target"
            return ControlDecision("hold", reason=self.hold_reason)
        self.safety_hold = False
        self.hold_reason = ""
        return ControlDecision("ee6d", row=tuple(float(value) for value in row), reason="tracking")

    def status(self, now_ns: int | None = None) -> dict[str, Any]:
        now_ns = int(now_ns or time.monotonic_ns())
        packet_age_ms = None
        tracking = {"left": False, "right": False, "head": False}
        if self.latest_packet is not None:
            packet_age_ms = max(0.0, (now_ns - self.latest_packet.received_monotonic_ns) / 1e6)
            tracking = {
                name: sample is not None and sample.tracking
                for name, sample in (
                    ("left", self.latest_packet.left),
                    ("right", self.latest_packet.right),
                    ("head", self.latest_packet.head),
                )
            }
        return {
            "session_state": self.session_state.value,
            "safety_hold": self.safety_hold,
            "hold_reason": self.hold_reason,
            "calibrated": self.calibration is not None,
            "last_sequence": self.last_sequence,
            "last_event_id": self.last_event_id,
            "packet_age_ms": None if packet_age_ms is None else round(packet_age_ms, 3),
            "source_tracking": tracking,
            "task": self.task,
            "scale": self.cfg.scale,
            "axis_map": self.cfg.axis_map,
            "orientation_mode": self.cfg.orientation_mode,
            "recording_gate": self.recording_gate,
            "last_error": self.last_error,
        }
