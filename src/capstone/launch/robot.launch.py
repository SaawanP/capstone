from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    robot_config = os.path.join(
        get_package_share_directory('capstone'),
        'config',
        'robot_launch.yaml'
    )

    device_controller = Node(
        package="capstone",
        executable="device_controller",
        parameters=[robot_config]
    )

    ld.add_action(device_controller)

    return ld