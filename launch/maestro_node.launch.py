from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_maestro',
            executable='maestro_node',
            name='maestro_motor_node',
            output='screen',
            parameters=[
                {'port': 'COM6'},
                {'baudrate': 9600},
                {'yaml_file': 'config/motor_ranges.yaml'}
            ]
        )
    ])
