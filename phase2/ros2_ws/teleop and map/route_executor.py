#!/usr/bin/env python3

# ================= ІМПОРТИ =================

import rclpy                          # Основна бібліотека ROS2
from rclpy.node import Node           # Базовий клас для ноди

from geometry_msgs.msg import Twist  # Повідомлення швидкостей

import yaml                           # Читання YAML-файлів маршрутів
import time                           # Робота з часом
import os                             # Робота з файловою системою


# =========================================
# Нода виконання маршруту з YAML-файлу
# =========================================

class RouteExecutor(Node):

    def __init__(self):

        super().__init__('route_executor')
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )


        # ================= ФАЙЛ МАРШРУТУ =================

        self.route_file = os.path.expanduser(
            '~/ros2_ws/routes/route.yaml'
        )
        # Завантажуємо всі кроки маршруту
        self.steps = self.load_route()


        # ================= СТАН ВИКОНАННЯ =================

        # Номер поточного кроку
        self.current_step = 0

        # Час початку поточного кроку
        self.step_start = time.time()


        # Таймер оновлення (20 Гц)
        self.timer = self.create_timer(
            0.05,
            self.update
        )

        self.get_logger().info("Route Executor started")


    # =============================
    # Завантаження YAML-маршруту

    def load_route(self):

        # Перевірка наявності файлу
        if not os.path.exists(self.route_file):

            self.get_logger().error(
                f"Route file not found: {self.route_file}"
            )

            # Якщо файлу немає — маршрут порожній
            return []


        # Відкриваємо YAML
        with open(self.route_file, 'r') as f:

            data = yaml.safe_load(f)


        # Повертаємо список кроків
        return data.get('steps', [])


    # =============================
    # Основний цикл виконання

    def update(self):

        # Якщо маршрут завершився
        if self.current_step >= len(self.steps):

            self.stop_robot()

            self.get_logger().info("Route finished")

            # Завершуємо ROS
            rclpy.shutdown()

            return


        # Поточний крок
        step = self.steps[self.current_step]

        # Тривалість кроку
        duration = step['time']

        now = time.time()


        # Якщо час кроку вийшов — переходимо далі
        if now - self.step_start >= duration:

            self.current_step += 1
            self.step_start = now
            return


        # ================= ФОРМУЄМО Twist =================

        msg = Twist()

        # Лінійні швидкості (x, y, z)
        msg.linear.x = step['linear'][0]
        msg.linear.y = step['linear'][1]
        msg.linear.z = step['linear'][2]

        # Кутові швидкості (x, y, z)
        msg.angular.x = step['angular'][0]
        msg.angular.y = step['angular'][1]
        msg.angular.z = step['angular'][2]


        # Публікація команди
        self.publisher.publish(msg)


    # =============================
    # Повна зупинка робота

    def stop_robot(self):

        # Порожній Twist = 0 швидкостей
        msg = Twist()

        self.publisher.publish(msg)



# =========================================
# MAIN
# =========================================

def main():

    # Ініціалізація ROS2
    rclpy.init()

    # Створення ноди
    node = RouteExecutor()

    try:
        # Запуск
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # Стоп перед виходом
        node.stop_robot()

        node.destroy_node()

        rclpy.shutdown()



# Точка входу
if __name__ == '__main__':
    main()
