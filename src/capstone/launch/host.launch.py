from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory("capstone")
    
    # Define config path
    host_config = os.path.join(
        pkg_dir,
        'config',
        'host_launch.yaml'
    )
    
    # Define the backend node

    brain_node = Node(
        package="capstone",
        executable="brain",
        parameters=[host_config]
    )
    
    # Define the joystick node
    joy_node = Node(
        package="joy",
        executable="joy_node",
    )
    
    # Define the path to the app's main.py
    app_main_path = os.path.join(pkg_dir, 'app', 'main.py')
    
    # Launch the app as a process
    app_process = ExecuteProcess(
        cmd=['python3', app_main_path],
        name='pyside6_app',
        output='screen'
    )
    
    # Create and return launch description
    ld = LaunchDescription()
    ld.add_action(joy_node)
    ld.add_action(app_process)
    ld.add_action(brain_node)

    return ld