from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Launch the remote subscriber on your Raspberry Pi
    remote_node = ExecuteProcess(
        cmd=[
            'ros2 run robot_actuators joy_subscriber'
        ],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([remote_node])

