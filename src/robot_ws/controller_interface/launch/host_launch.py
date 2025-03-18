from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Launch the standard joy node which reads your Logitech F710
    joy_node = ExecuteProcess(
        cmd=['ros2', 'run', 'joy', 'joy_node'],
        output='screen'
    )
    
    return LaunchDescription([joy_node])

