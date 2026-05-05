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

                # ВАЖЛИВО:
                # 0 градусів лідара має бути прямо перед роботом.
                'angle_offset_deg': 0.0,
                'invert_angle_direction': False,

                'front_min_deg': -35.0,
                'front_max_deg': 35.0,
                'front_min_points': 1,
                'min_confidence': 0,
            }]
        ),

        Node(
            package='robot_bringup',
            executable='camera_node',
            name='aruco_detector_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device': '/dev/video0',
            }]
        ),

        Node(
            package='robot_bringup',
            executable='mission_auto',
            name='mission_auto',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'aruco_topic': '/aruco/target',
                'scan_topic': '/scan',
                'cmd_vel_topic': '/cmd_vel',

                # Якщо потрібно пройти конкретні маркери:
                # 'target_ids': '1,2,3',
                # Якщо пусто — будь-який новий маркер.
                'target_ids': '',

                'stop_distance_m': 0.24,
                'slowdown_distance_m': 0.55,
                'obstacle_trigger_m': 0.42,
                'obstacle_clear_m': 0.55,

                'track_linear_fast': 0.09,
                'track_linear_slow': 0.045,

                'search_angular_z': 0.20,

                'kp_angular': 0.003,
                'max_track_angular_z': 0.50,

                'avoid_turn_angular_z': 0.40,
                'avoid_forward_linear_x': 0.06,
                'avoid_curve_angular_z': 0.15,
            }]
        ),

        Node(
            package='robot_bringup',
            executable='motor_node',
            name='diff_drive_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
