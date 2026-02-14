#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

# Настройки скорости
LINEAR_STEP = 0.05
ANGULAR_STEP = 0.2
MAX_LINEAR = 0.25
MAX_ANGULAR = 2.0

# ========================================

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')

        # Publisher на /cmd_vel
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.v = 0.0
        self.w = 0.0

        self.get_logger().info(
            "Keyboard Teleop Ready:\n"
            "W/S: вперед/назад, A/D: поворот, Z/C: разворот на месте, Q: стоп"
        )

        # Настройка терминала для неблокирующего ввода
        self.settings = termios.tcgetattr(sys.stdin)

        self.timer = self.create_timer(0.05, self.update)  # 20Hz публикация

    def get_key(self):
        """Возвращает нажатую клавишу без блокировки"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = ''
        if rlist:
            key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def update(self):
        key = self.get_key()
        twist = Twist()

        # ===== Управление =====
        if key.lower() == 'w':
            self.v += LINEAR_STEP
        elif key.lower() == 's':
            self.v -= LINEAR_STEP
        elif key.lower() == 'a':
            self.w += ANGULAR_STEP
        elif key.lower() == 'd':
            self.w -= ANGULAR_STEP
        elif key.lower() == 'z':  # разворот на месте влево
            self.v = 0.0
            self.w = MAX_ANGULAR
        elif key.lower() == 'c':  # разворот на месте вправо
            self.v = 0.0
            self.w = -MAX_ANGULAR
        elif key.lower() == 'q':  # стоп
            self.v = 0.0
            self.w = 0.0
        elif key == '\x03':  # Ctrl-C
            self.get_logger().info("Exiting Teleop")
            rclpy.shutdown()
            return

        # Ограничения
        self.v = max(-MAX_LINEAR, min(MAX_LINEAR, self.v))
        self.w = max(-MAX_ANGULAR, min(MAX_ANGULAR, self.w))

        # Формируем Twist
        twist.linear.x = self.v
        twist.angular.z = self.w

        # Публикуем команду
        self.pub.publish(twist)


# ========================================

def main():
    rclpy.init()
    node = KeyboardTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
