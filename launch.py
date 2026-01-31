from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='diff_drive_l298n',
            executable='diff_drive_node',
            name='diff_drive_l298n',
            output='screen',
        )
    ])
