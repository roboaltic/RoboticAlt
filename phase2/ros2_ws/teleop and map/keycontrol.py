#!/usr/bin/env python3

# ================= ІМПОРТИ =================

import rclpy                         # Основна бібліотека ROS2
from rclpy.node import Node          # Базовий клас для ноди

from geometry_msgs.msg import Twist # Повідомлення для керування швидкістю

import sys                           # Доступ до stdin
import termios                       # Налаштування терміналу (Linux)
import tty                           # Raw-режим клавіатури
import select                        # Неблокуюче читання


# ================= НАЛАШТУВАННЯ ШВИДКОСТІ =================

LINEAR_STEP = 0.055
ANGULAR_STEP = 0.2
MAX_LINEAR = 0.25
MAX_ANGULAR = 2.0


# ========================================


# Нода для керування роботом з клавіатури
class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop_node')
        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Поточні швидкості
        self.v = 0.0   # лінійна
        self.w = 0.0   # кутова

        # Інструкція для користувача
        self.get_logger().info(
            "Keyboard Teleop Ready:\n"
            "W/S: вперед/назад\n"
            "A/D: поворот\n"
            "Z/C: розворот на місці\n"
            "Q: стоп"
        )

        # Збереження поточних налаштувань терміналу
        self.settings = termios.tcgetattr(sys.stdin)

        # Таймер оновлення (20 Гц)
        self.timer = self.create_timer(
            0.05,
            self.update
        )


    # ========================================
    # Зчитування клавіші без блокування

    def get_key(self):
        """
        Повертає натиснуту клавішу без блокування програми
        """

        # Переводимо термінал у raw-режим
        tty.setraw(sys.stdin.fileno())

        # Перевіряємо, чи є введення
        rlist, _, _ = select.select(
            [sys.stdin],
            [],
            [],
            0.1
        )

        key = ''

        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b':
                key += sys.stdin.read(2)

        # Повертаємо стандартні налаштування терміналу
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            self.settings
        )

        return key


    # ========================================
    # Головний цикл керування

    def update(self):

        # Отримуємо натиснуту клавішу
        key = self.get_key()

        # Створюємо повідомлення Twist
        twist = Twist()


        # ========== КЕРУВАННЯ ==========

        if key.lower() == '\x1b[Aa':          # Вперед
            self.v -= LINEAR_STEP

        elif key.lower() == '\x1b[B':        # Назад
            self.v += LINEAR_STEP

        elif key.lower() == '\x1b[C':        # Поворот вліво
            self.w += ANGULAR_STEP

        elif key.lower() == '\x1b[D':        # Поворот вправо
            self.w -= ANGULAR_STEP

        elif key.lower() == 'z':        # Розворот вліво на місці
            self.v = 0.0
            self.w = MAX_ANGULAR

        elif key.lower() == 'c':        # Розворот вправо на місці
            self.v = 0.0
            self.w = -MAX_ANGULAR

        elif key.lower() == 'q':        # Стоп
            self.v = 0.0
            self.w = 0.0

        elif key == '\x03':             # Ctrl + C
            self.get_logger().info("Exiting Teleop")
            rclpy.shutdown()
            return


        # ========== ОБМЕЖЕННЯ ШВИДКОСТІ ==========

        self.v = max(-MAX_LINEAR,
                     min(MAX_LINEAR, self.v))

        self.w = max(-MAX_ANGULAR,
                     min(MAX_ANGULAR, self.w))


        # ========== ФОРМУВАННЯ Twist ==========

        twist.linear.x = self.v
        twist.angular.z = self.w


        # ========== ПУБЛІКАЦІЯ ==========

        self.pub.publish(twist)


# ========================================
# MAIN

def main():

    # Ініціалізація ROS2
    rclpy.init()

    # Створення ноди
    node = KeyboardTeleop()

    try:
        # Запуск
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # Коректне завершення
        node.destroy_node()
        rclpy.shutdown()


# Точка входу
if __name__ == '__main__':
    main()
