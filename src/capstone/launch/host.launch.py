from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    host_config = os.path.join(
        get_package_share_directory("capstone"),
        'config',
        'host_launch.yaml'
    )

    brain_node = Node(
        package="capstone",
        executable="brain",
        parameters=[host_config]
    )

    backend_node = Node(
        package="capstone",
        executable="backend",
        parameters=[host_config]
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
    )

    ld.add_action(backend_node)
    ld.add_action(joy_node)
    ld.add_action(brain_node)

    return ld