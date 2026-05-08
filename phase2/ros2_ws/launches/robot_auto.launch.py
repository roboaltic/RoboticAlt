from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # --- 1. ЛИДАР ---
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
                'min_confidence': 0,
                'angle_resolution_deg': 1.0,
                'angle_offset_deg': 0.0,
                'invert_angle_direction': False,
                'front_min_deg': -45.0,
                'front_max_deg': 45.0,
                'point_max_age_sec': 0.35,
                'front_min_points': 1,
                'front_hold_sec': 0.8,
                'front_filter_window': 3,
            }]
        ),

        # --- 2. СИСТЕМА БЕЗОПАСНОСТИ ЛИДАРА ---
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

        # --- 3. МОТОРЫ ---
        Node(
            package='diff_drive_l298n',
            executable='diff_drive_node',
            name='motor_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'cmd_vel_topic': '/cmd_vel',
            }]
        ),

        # ==========================================
        # --- НОВЫЕ НОДЫ ДЛЯ КАМЕРЫ И УПРАВЛЕНИЯ ---
        # ==========================================

        # --- 4. ДРАЙВЕР КАМЕРЫ ---
        Node(
            package='v4l2_camera', # Стандартный пакет ROS 2 для веб-камер
            executable='v4l2_camera_node',
            name='camera_driver',
            output='screen',
            parameters=[{
                'video_device': '/dev/video2', # Проверь, чтобы порт совпадал с твоей камерой
                'image_size': [640, 480]
            }],
            # Если твои скрипты ждут топик /camera/image_raw, делаем ремаппинг:
            remappings=[
                ('/image_raw', '/camera/image_raw')
            ]
        ),

        # --- 5. АНАЛИТИКА (Трещины и контуры) ---
        Node(
            package='camera', # Имя пакета, где лежит vision_processing_node.py
            executable='vision_processing_node',
            name='vision_node',
            output='screen'
        ),

        # --- 6. НАВИГАЦИЯ ARUCO ---
        Node(
            package='camera', # Имя пакета, где лежит aruco_navigation_node.py
            executable='aruco_navigation_node',
            name='navigation_node',
            output='screen'
        ),

        # --- 7. FLASK ВЕБ-СЕРВЕР ---
        Node(
            package='camera', # Имя пакета, где лежит flask_web_node.py
            executable='flask_web_node',
            name='flask_web_ui',
            output='screen'
        )
    ])