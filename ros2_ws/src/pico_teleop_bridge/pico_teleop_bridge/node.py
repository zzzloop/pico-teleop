"""ROS 2 node that forwards standard PICO topics to Isaac Lab over UDP."""

from __future__ import annotations

import json
import socket
import time
from collections import deque
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger


PROTOCOL_VERSION = 1


class PicoTeleopBridge(Node):
    def __init__(self) -> None:
        super().__init__("pico_teleop_bridge")
        self.declare_parameter("sim_port", 9765)
        self.declare_parameter("packet_hz", 60.0)
        self.declare_parameter("pose_timeout_s", 0.25)
        self.declare_parameter("task", "Teleoperate BRX042501 with PICO controllers.")
        self.declare_parameter("button_start", 0)
        self.declare_parameter("button_pause", 1)
        self.declare_parameter("button_record", 2)
        self.declare_parameter("button_stop", 3)
        self.declare_parameter("button_calibrate", 4)

        # Bridge and simulator are required to run on the same server. Do not
        # make this a configurable LAN endpoint.
        self.sim_address = ("127.0.0.1", int(self.get_parameter("sim_port").value))
        self.pose_timeout_s = float(self.get_parameter("pose_timeout_s").value)
        self.task = str(self.get_parameter("task").value)
        self.sequence = 0
        self.pose: dict[str, tuple[dict[str, Any], int]] = {}
        self.axes: list[float] = []
        self.buttons: list[int] = []
        self.previous_buttons: list[int] = []
        self.events: deque[dict[str, Any]] = deque()
        # Wall-clock nanoseconds keep IDs monotonic across bridge restarts, so
        # a running simulator never mistakes a new event for an old duplicate.
        self.event_sequence = time.time_ns()
        self.last_sim_status: dict[str, Any] = {}
        self.last_sim_status_ns = 0

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", 0))
        self.socket.setblocking(False)

        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(PoseStamped, "/pico/left_controller/pose", self._pose_callback("left"), pose_qos)
        self.create_subscription(PoseStamped, "/pico/right_controller/pose", self._pose_callback("right"), pose_qos)
        self.create_subscription(PoseStamped, "/pico/head/pose", self._pose_callback("head"), pose_qos)
        self.create_subscription(Joy, "/pico/controllers/joy", self._joy_callback, reliable_qos)
        self.create_subscription(String, "/pico_teleop/task", self._task_callback, reliable_qos)
        self.status_publisher = self.create_publisher(String, "/pico_teleop/status", status_qos)
        self.joint_publisher = self.create_publisher(JointState, "/pico_teleop/joint_states", reliable_qos)

        for command in (
            "start", "pause", "resume", "stop", "reset", "calibrate",
            "record_start", "record_stop", "record_abort", "record_finalize",
        ):
            self.create_service(Trigger, f"/pico_teleop/{command}", self._service_callback(command))

        packet_hz = max(float(self.get_parameter("packet_hz").value), 1.0)
        self.create_timer(1.0 / packet_hz, self._tick)
        self.get_logger().info(
            f"PICO bridge ready: ROS 2 -> udp://{self.sim_address[0]}:{self.sim_address[1]} at {packet_hz:.1f} Hz"
        )

    def _pose_callback(self, name: str):
        def callback(message: PoseStamped) -> None:
            self.pose[name] = (
                {
                    "position": [
                        float(message.pose.position.x),
                        float(message.pose.position.y),
                        float(message.pose.position.z),
                    ],
                    "quaternion": [
                        float(message.pose.orientation.x),
                        float(message.pose.orientation.y),
                        float(message.pose.orientation.z),
                        float(message.pose.orientation.w),
                    ],
                    "tracking": True,
                },
                time.monotonic_ns(),
            )
        return callback

    def _task_callback(self, message: String) -> None:
        task = message.data.strip()
        if task:
            self.task = task

    @staticmethod
    def _pressed(buttons: list[int], index: int) -> bool:
        return 0 <= index < len(buttons) and bool(buttons[index])

    def _joy_callback(self, message: Joy) -> None:
        self.axes = [float(value) for value in message.axes]
        current = [int(bool(value)) for value in message.buttons]
        mapping = {
            int(self.get_parameter("button_start").value): "start",
            int(self.get_parameter("button_pause").value): "pause",
            int(self.get_parameter("button_record").value): "record_start",
            int(self.get_parameter("button_stop").value): "stop",
            int(self.get_parameter("button_calibrate").value): "calibrate",
        }
        teleop_status = self.last_sim_status.get("teleop", {})
        recorder_status = self.last_sim_status.get("recorder", {})
        for index, command in mapping.items():
            if not self._pressed(current, index) or self._pressed(self.previous_buttons, index):
                continue
            if command == "start" and teleop_status.get("session_state") == "paused":
                command = "resume"
            if command == "record_start" and recorder_status.get("active"):
                command = "record_stop"
            self._queue_event(command)
        self.previous_buttons = current
        self.buttons = current

    def _queue_event(self, command: str) -> None:
        self.event_sequence += 1
        event = {"command": command, "event_id": self.event_sequence}
        if command in ("start", "record_start"):
            event["task"] = self.task
        self.events.append(event)
        while len(self.events) > 64:
            dropped = self.events.popleft()
            self.get_logger().error(f"Dropped unacknowledged teleop event: {dropped}")

    def _service_callback(self, command: str):
        def callback(_request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
            self._queue_event(command)
            response.success = True
            response.message = f"queued {command}; observe /pico_teleop/status for completion"
            return response
        return callback

    def _current_pose(self, name: str, now_ns: int) -> dict[str, Any] | None:
        item = self.pose.get(name)
        if item is None:
            return None
        value, received_ns = item
        result = dict(value)
        result["tracking"] = (now_ns - received_ns) / 1e9 <= self.pose_timeout_s
        return result

    def _receive_status(self) -> None:
        while True:
            try:
                payload, _sender = self.socket.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError as exc:
                self.get_logger().warning(f"UDP status receive failed: {exc}")
                return
            try:
                value = json.loads(payload.decode("utf-8"))
                if value.get("type") != "status" or int(value.get("protocol", -1)) != PROTOCOL_VERSION:
                    continue
                self.last_sim_status = dict(value.get("status", {}))
                self.last_sim_status_ns = time.monotonic_ns()
                acknowledged = int(self.last_sim_status.get("teleop", {}).get("last_event_id", 0))
                while self.events and int(self.events[0].get("event_id", 0)) <= acknowledged:
                    self.events.popleft()
                status_message = String()
                status_message.data = json.dumps(self.last_sim_status, ensure_ascii=False, separators=(",", ":"))
                self.status_publisher.publish(status_message)
                if "qpos23" in self.last_sim_status:
                    joint = JointState()
                    joint.header.stamp = self.get_clock().now().to_msg()
                    joint.name = list(self.last_sim_status.get("joint_names23", ()))
                    joint.position = [float(value) for value in self.last_sim_status["qpos23"]]
                    self.joint_publisher.publish(joint)
            except Exception as exc:
                self.get_logger().warning(f"Rejected simulator status: {exc}")

    def _tick(self) -> None:
        self._receive_status()
        now_ns = time.monotonic_ns()
        self.sequence += 1
        packet: dict[str, Any] = {
            "type": "teleop",
            "protocol": PROTOCOL_VERSION,
            "sequence": self.sequence,
            "sent_monotonic_ns": now_ns,
            "axes": self.axes,
            "buttons": self.buttons,
            # Events stay in every packet until the simulator acknowledges
            # teleop.last_event_id. This makes start/pause/stop reliable over UDP.
            "events": list(self.events),
        }
        for name in ("left", "right", "head"):
            pose = self._current_pose(name, now_ns)
            if pose is not None:
                packet[name] = pose
        try:
            self.socket.sendto(json.dumps(packet, separators=(",", ":")).encode("utf-8"), self.sim_address)
        except OSError as exc:
            self.get_logger().error(f"UDP send failed: {exc}")

    def destroy_node(self) -> bool:
        self.socket.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PicoTeleopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
