#!/usr/bin/env python3

# ================= ІМПОРТИ =================

import rclpy                          # Основна бібліотека ROS2
from rclpy.node import Node           # Базовий клас для ноди

from geometry_msgs.msg import Twist  # Повідомлення швидкостей (/cmd_vel)
from nav_msgs.msg import Odometry    # Повідомлення одометрії

import lgpio                          # Бібліотека керування GPIO (Raspberry Pi)
import time                           # Робота з часом
import math                           # Математичні функції


# ================= GPIO (ПІНИ) =================
# Піни для драйвера L298N

# Лівий мотор
ENA = 18      # PWM (швидкість)
IN1 = 23      # Напрямок
IN2 = 24      # Напрямок

# Правий мотор
ENB = 13      # PWM (швидкість)
IN3 = 27      # Напрямок
IN4 = 22      # Напрямок

# Частота PWM (Гц)
PWM_FREQ = 1000


# ================= ПАРАМЕТРИ РОБОТА =================

# Відстань між колесами (метри)
WHEEL_BASE = 0.16

# Максимальна лінійна швидкість (м/с)
MAX_LINEAR = 0.25

# Максимальна кутова швидкість (рад/с)
MAX_ANGULAR = 2.0

# Таймаут команд (сек)
# Якщо команда не приходить — робот зупиняється
CMD_TIMEOUT = 0.5


# ================================================


# Клас ноди диференціального приводу
class DiffDrive(Node):

    def __init__(self):

        # Ініціалізація ноди
        super().__init__('diff_drive_node')

        self.get_logger().info("Starting DiffDrive (lgpio / chip4)")


        # ================= ROS2 =================

        # Підписка на /cmd_vel
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_cb,
            10
        )

        # Publisher одометрії
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        # Таймер оновлення (20 Гц)
        self.timer = self.create_timer(0.05, self.update)


        # ================= GPIO =================

        # Відкриваємо GPIO-чіп №4 (для RPi5)
        self.chip = lgpio.gpiochip_open(4)

        # Налаштовуємо піни
        self.setup_gpio()


        # ================= СТАН РОБОТА =================

        # Лінійна швидкість
        self.v = 0.0

        # Кутова швидкість
        self.w = 0.0

        # Час останньої команди
        self.last_cmd = time.time()

        # Позиція робота
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0   # Орієнтація

        # Час останнього оновлення
        self.last_time = time.time()

        self.get_logger().info("DiffDrive Ready")


    # ========================================
    # Налаштування GPIO

    def setup_gpio(self):

        # Список усіх пінів
        pins = [ENA, IN1, IN2, ENB, IN3, IN4]

        # Робимо їх виходами
        for p in pins:
            lgpio.gpio_claim_output(self.chip, p)

        # Зупиняємо мотори
        self.stop()


    # ========================================
    # Callback для /cmd_vel

    def cmd_cb(self, msg: Twist):

        # Обмежуємо швидкості
        self.v = max(-MAX_LINEAR,
                     min(MAX_LINEAR, msg.linear.x))

        self.w = max(-MAX_ANGULAR,
                     min(MAX_ANGULAR, msg.angular.z))

        # Запам’ятовуємо час команди
        self.last_cmd = time.time()


    # ========================================
    # Керування лівим мотором

    def set_left(self, s):

        # PWM у %
        duty = int(abs(s) * 100)
        duty = max(0, min(100, duty))

        # Напрямок
        if s >= 0:
            lgpio.gpio_write(self.chip, IN1, 1)
            lgpio.gpio_write(self.chip, IN2, 0)
        else:
            lgpio.gpio_write(self.chip, IN1, 0)
            lgpio.gpio_write(self.chip, IN2, 1)

        # Встановлення PWM
        lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, duty)


    # ========================================
    # Керування правим мотором

    def set_right(self, s):

        duty = int(abs(s) * 100)
        duty = max(0, min(100, duty))

        if s >= 0:
            lgpio.gpio_write(self.chip, IN3, 1)
            lgpio.gpio_write(self.chip, IN4, 0)
        else:
            lgpio.gpio_write(self.chip, IN3, 0)
            lgpio.gpio_write(self.chip, IN4, 1)

        lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, duty)


    # ========================================
    # Повна зупинка моторів

    def stop(self):

        # PWM = 0
        lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, 0)
        lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, 0)

        # Всі напрямки = 0
        for p in [IN1, IN2, IN3, IN4]:
            lgpio.gpio_write(self.chip, p, 0)


    # ========================================
    # Головне оновлення (20 Гц)

    def update(self):

        now = time.time()

        # Час між оновленнями
        dt = now - self.last_time
        self.last_time = now


        # ================= Watchdog =================

        # Якщо давно не було команд — стоп
        if now - self.last_cmd > CMD_TIMEOUT:
            self.v = 0.0
            self.w = 0.0


        # ================= Кінематика =================

        # Швидкості коліс
        vl = self.v - self.w * WHEEL_BASE / 2
        vr = self.v + self.w * WHEEL_BASE / 2


        # Нормалізація
        maxv = max(abs(vl), abs(vr))

        if maxv > MAX_LINEAR:
            k = MAX_LINEAR / maxv
            vl *= k
            vr *= k


        # Значення від -1 до 1
        left = vl / MAX_LINEAR
        right = vr / MAX_LINEAR


        # ================= Мотори =================

        self.set_left(left)
        self.set_right(right)


        # ================= Одометрія =================

        v = (vl + vr) / 2
        w = (vr - vl) / WHEEL_BASE

        self.theta += w * dt

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        self.publish_odom(v, w)


    # ========================================
    # Публікація /odom

    def publish_odom(self, v, w):

        msg = Odometry()

        # Час
        msg.header.stamp = self.get_clock().now().to_msg()

        # Фрейми
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # Позиція
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y

        # Орієнтація (quaternion)
        msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        msg.pose.pose.orientation.w = math.cos(self.theta / 2)

        # Швидкість
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        # Публікація
        self.odom_pub.publish(msg)


    # ========================================
    # Коректне завершення

    def destroy_node(self):

        # Зупинка моторів
        self.stop()

        # Закриття GPIO
        lgpio.gpiochip_close(self.chip)

        super().destroy_node()


# ========================================
# MAIN

def main():

    # Ініціалізація ROS2
    rclpy.init()

    # Створення ноди
    node = DiffDrive()

    try:
        # Запуск
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # Завершення
        node.destroy_node()
        rclpy.shutdown()


# Точка входу
if __name__ == '__main__':
    main()
