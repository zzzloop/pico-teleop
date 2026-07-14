from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution([FindPackageShare("pico_teleop_bridge"), "config", "default.yaml"])
    return LaunchDescription(
        [
            Node(
                package="ros_tcp_endpoint",
                executable="default_server_endpoint",
                name="ros_tcp_endpoint",
                output="screen",
                parameters=[{"ROS_IP": "0.0.0.0", "ROS_TCP_PORT": 10000}],
            ),
            Node(
                package="pico_teleop_bridge",
                executable="bridge",
                name="pico_teleop_bridge",
                output="screen",
                parameters=[config],
            )
        ]
    )
