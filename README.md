# ROS2 Robot (Raspberry Pi)

Цей проєкт реалізує мобільного робота на базі ROS2 та Raspberry Pi 
з підтримкою:

- керування DC-моторами;
- підключення камери;
- відеостриму;
- автоматичного руху за маршрутом;
- ручного керування через `/cmd_vel`.

---

## Можливості системи

- Керування моторами через GPIO + PWM
- Публікація швидкостей у `/cmd_vel`
- Підтримка камери
- Відеотрансляція
- Запуск усієї системи через launch-файл
- Виконання маршрутів із YAML-файлів
- Циклічний рух
- Безпечна зупинка


## Використані технології

- ROS2 (Jazzy)
- Python 3
- Raspberry Pi 5
- L298N
- YAML
- OpenCV

## Ручне керування роботом
~/ros2_ws/colcon build --symlink-install
~/ros2_ws/source install/setup.bash
ros2 launch robot_bringup system.launch.py
рух вперед
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: -0.1
angular:
  z: 0.0"

рух назад
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: 0.1
angular:
  z: 0.0"

рух вправо
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: -0.09
angular:
  z: -1.5"

рухв вліво
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: -0.09
angular:
  z: 1.5"
