from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    camera_node = Node(
        package='camera_driver',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[
            {
                'device': '/dev/video0',
                'fps': 15,
                'width': 640,
                'height': 480
            }
        ]
    )

    motor_node = Node(
        package='motor_driver',
        executable='diff_drive_l298n, diff_drive_node',
        name='diff_drive_l298n',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        camera_node,
        motor_node
    ])
