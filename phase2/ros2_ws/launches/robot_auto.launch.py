from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='robot_bringup',
            executable='ld06_node',
            name='ld06_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 230400,
                'frame_id': 'laser_frame',

                'scan_topic': '/scan',
                'front_distance_topic': '/lidar/front_distance',
                'closest_topic': '/lidar/closest',

                'publish_rate_hz': 40.0,

                'range_min_m': 0.05,
                'range_max_m': 12.0,

                # Для діагностики постав 0.
                # Коли все буде стабільно, можна 5.
                'min_confidence': 0,

                'angle_resolution_deg': 1.0,

                # Калібрується після запуску.
                'angle_offset_deg': 0.0,
                'invert_angle_direction': False,

                # Передній сектор.
                'front_min_deg': -45.0,
                'front_max_deg': 45.0,

                'point_max_age_sec': 0.35,
                'front_min_points': 1,
                'front_hold_sec': 0.8,
                'front_filter_window': 3,
            }]
        ),

        Node(
            package='robot_bringup',
            executable='lidar_safety',
            name='lidar_safety',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'input_cmd_topic': '/cmd_vel_raw',
                'output_cmd_topic': '/cmd_vel',
                'front_distance_topic': '/lidar/front_distance',

                'cmd_timeout_sec': 0.5,
                'lidar_timeout_sec': 0.7,

                # Зупинка.
                'stop_distance_m': 0.45,
                'clear_distance_m': 0.50,

                'allow_rotation_when_blocked': True,
                'allow_reverse_when_blocked': True,

                'max_linear_x': 0.35,
                'max_angular_z': 1.5,

                'enable_slowdown': False,
                'slowdown_distance_m': 0.80,
                'min_slowdown_factor': 0.25,

                'invalid_front_timeout_sec': 0.6,
            }]
        ),

        Node(
            package='diff_drive_l298n',
            executable='diff_drive_node',
            name='motor_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                # Перевір, щоб твоя motor_node реально мала такий параметр.
                # Якщо ні — у motor_node треба підписку зробити на /cmd_vel.
                'cmd_vel_topic': '/cmd_vel',
            }]
        ),

    ])
