#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import lgpio
import time
import math


ENA = 17
IN1 = 27
IN2 = 22

ENB = 13
IN3 = 26
IN4 = 19

PWM_FREQ = 700

# Мінімальна потужність PWM у %
# Якщо мотори пищать, але не рушають — підніми до 55 або 65
MIN_PWM = 90

WHEEL_BASE = 0.16
MAX_LINEAR = 0.2
MAX_ANGULAR = 2.0

CMD_TIMEOUT = 0.5


class DiffDrive(Node):

    def __init__(self):
        super().__init__('diff_drive_node')

        self.get_logger().info("Starting DiffDrive")

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_cb,
            1
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.timer = self.create_timer(0.01, self.update)

        self.chip = lgpio.gpiochip_open(4)

        self.setup_gpio()

        self.v = 0.0
        self.w = 0.0
        self.last_cmd = time.time()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = time.time()

        self.get_logger().info("DiffDrive Ready. Listening /cmd_vel")

    def setup_gpio(self):
        pins = [ENA, IN1, IN2, ENB, IN3, IN4]

        for p in pins:
            lgpio.gpio_claim_output(self.chip, p)

        self.stop()

    def cmd_cb(self, msg: Twist):
        self.v = max(-MAX_LINEAR, min(MAX_LINEAR, msg.linear.x))
        self.w = max(-MAX_ANGULAR, min(MAX_ANGULAR, msg.angular.z))
        self.last_cmd = time.time()

        # Миттєва реакція на стоп-команду від lidar_safety
        if abs(self.v) < 0.001 and abs(self.w) < 0.001:
            self.stop()

    def pwm_from_speed(self, s):
        duty = int(abs(s) * 100)

        if duty > 0:
            duty = max(MIN_PWM, duty)

        duty = max(0, min(100, duty))
        return duty

    def set_left(self, s):
        duty = self.pwm_from_speed(s)

        if s > 0:
            lgpio.gpio_write(self.chip, IN1, 1)
            lgpio.gpio_write(self.chip, IN2, 0)
        elif s < 0:
            lgpio.gpio_write(self.chip, IN1, 0)
            lgpio.gpio_write(self.chip, IN2, 1)
        else:
            lgpio.gpio_write(self.chip, IN1, 0)
            lgpio.gpio_write(self.chip, IN2, 0)

        lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, duty)

    def set_right(self, s):
        duty = self.pwm_from_speed(s)

        if s > 0:
            lgpio.gpio_write(self.chip, IN3, 1)
            lgpio.gpio_write(self.chip, IN4, 0)
        elif s < 0:
            lgpio.gpio_write(self.chip, IN3, 0)
            lgpio.gpio_write(self.chip, IN4, 1)
        else:
            lgpio.gpio_write(self.chip, IN3, 0)
            lgpio.gpio_write(self.chip, IN4, 0)

        lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, duty)

    def stop(self):
        lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, 0)
        lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, 0)

        for p in [IN1, IN2, IN3, IN4]:
            lgpio.gpio_write(self.chip, p, 0)

    def update(self):
        now = time.time()

        dt = now - self.last_time
        self.last_time = now

        if now - self.last_cmd > CMD_TIMEOUT:
            self.v = 0.0
            self.w = 0.0

        vl = self.v - self.w * WHEEL_BASE / 2
        vr = self.v + self.w * WHEEL_BASE / 2

        maxv = max(abs(vl), abs(vr))

        if maxv > MAX_LINEAR:
            k = MAX_LINEAR / maxv
            vl *= k
            vr *= k

        left = vl / MAX_LINEAR
        right = vr / MAX_LINEAR

        self.set_left(left)
        self.set_right(right)

        v = (vl + vr) / 2
        w = (vr - vl) / WHEEL_BASE

        self.theta += w * dt

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        self.publish_odom(v, w)

    def publish_odom(self, v, w):
        msg = Odometry()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y

        msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        msg.pose.pose.orientation.w = math.cos(self.theta / 2)

        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        self.odom_pub.publish(msg)

    def destroy_node(self):
        try:
            self.stop()
            lgpio.gpiochip_close(self.chip)
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = DiffDrive()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
