from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Launch the standard joy node which reads your Logitech F710
    joy_node = ExecuteProcess(
        cmd=['ros2', 'run', 'joy', 'joy_node'],
        output='screen'
    )

    # Launch the remote subscriber on your Raspberry Pi
    remote_node = ExecuteProcess(
        cmd=[
            'ssh', 'pi@192.168.1.2',
            'source ~/capstone/src/robot_ws/install/setup.bash && '
            'ros2 run robot_actuators joy_subscriber'
        ],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([joy_node, remote_node])

