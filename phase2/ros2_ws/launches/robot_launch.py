# Імпорт класу LaunchDescription — описує набір нод для запуску
from launch import LaunchDescription

# Імпорт Node — дозволяє запускати ROS2-ноди
from launch_ros.actions import Node


# Головна функція, яку викликає ros2 launch
def generate_launch_description():

    # === Опис ноди камери ===
    camera_node = Node(
        package='camera_driver',
        executable='camera_node',
        name='camera_node',
        output='screen',

        # Передача параметрів у ноду
        parameters=[
            {
                'device': '/dev/video0',
                'fps': 15,
                'width': 640,
                'height': 480
            }
        ]
    )

    # === Опис ноди керування моторами ===
    motor_node = Node(
        package='motor_driver',
        executable='diff_drive_l298n',
        name='diff_drive_l298n',
        output='screen',
        emulate_tty=True
    )

    # === Повертаємо список нод для запуску ===
    return LaunchDescription([
        camera_node,
        motor_node
    ])
