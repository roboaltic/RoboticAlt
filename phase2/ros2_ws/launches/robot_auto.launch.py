from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ---------------- LD06 LIDAR ----------------
        Node(
            package='robot_bringup',
            executable='ld06_node',
            name='ld06_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 230400,
            }]
        ),

        # ---------------- KEYBOARD TELEOP ----------------
        Node(
            package='diff_drive_l298n',
            executable='keyboard_teleop_node',
            name='keyboard_teleop_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'cmd_vel_topic': '/cmd_vel_raw',
                'linear_speed': 0.50,
                'angular_speed': 1.7,
            }]
        ),

        # ---------------- LIDAR OBSTACLE AVOIDANCE ----------------
        Node(
            package='robot_bringup',
            executable='lidar_obstacle_node',
            name='lidar_obstacle_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'scan_topic': '/scan',
                'cmd_vel_topic': '/cmd_vel_raw',

                # False — щоб авто-нода не заважала клавіатурі
                'auto_start': False,

                'forward_speed': 0.5,
                

                'turn_speed': 1.7,
                'wall_follow_speed': 0.09,

                'obstacle_dist': 0.55,
                'clear_dist': 0.85,
                'side_desired_dist': 0.30,
                'side_open_dist': 0.85,

                'wall_kp': 1.4,
                'return_gain': 1.0,

                'control_period': 0.05,
                'exit_confirm_cycles': 8,
            }]
        ),

        # ---------------- SAFETY FILTER ----------------
        Node(
            package='robot_bringup',
            executable='lidar_safety_node',
            name='lidar_safety_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'scan_topic': '/scan',
                'input_cmd_topic': '/cmd_vel_raw',
                'output_cmd_topic': '/cmd_vel',
                'stop_dist': 0.15,
                'front_angle_deg': 35.0,
            }]
        ),

        # ---------------- MOTORS ----------------
        Node(
            package='diff_drive_l298n',
            executable='diff_drive_node',
            name='diff_drive_node',
            output='screen',
            emulate_tty=True
        ),
    ])
