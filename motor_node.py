#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import lgpio
import time
import math


# ================== GPIO CONFIG ==================

ENA = 18
IN1 = 23
IN2 = 24

ENB = 13
IN3 = 27
IN4 = 22

PWM_FREQ = 1000


# ================== ROBOT PARAMS ==================

WHEEL_RADIUS = 0.033
WHEEL_BASE = 0.16

MAX_LINEAR = 0.25
MAX_ANGULAR = 2.0

CMD_TIMEOUT = 0.5


# =============== CURRENT PROTECTION ===============

STALL_CURRENT = 1.8      # Амперы (подстрой!)
STALL_TIME = 0.7         # сек
RECOVERY_TIME = 2.0      # сек


# ==================================================


class DiffDrive(Node):

    def __init__(self):
        super().__init__('diff_drive_node')

        self.get_logger().info("Starting DiffDrive L298N Node")

        # ROS
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_cb,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.timer = self.create_timer(0.05, self.update)

        # GPIO
        self.chip = lgpio.gpiochip_open(0)

        self._setup_gpio()

        # State
        self.v = 0.0
        self.w = 0.0

        self.last_cmd_time = time.time()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = time.time()

        # Protection
        self.stall_start = None
        self.emergency = False
        self.recover_time = 0.0

        self.get_logger().info("DiffDrive Ready")


    # ==================================================

    def _setup_gpio(self):

        pins = [ENA, IN1, IN2, ENB, IN3, IN4]

        for p in pins:
            lgpio.gpio_claim_output(self.chip, p, 0)

        self.stop_motors()


    # ==================================================

    def cmd_cb(self, msg: Twist):

        if self.emergency:
            return

        v = msg.linear.x
        w = msg.angular.z

        v = max(-MAX_LINEAR, min(MAX_LINEAR, v))
        w = max(-MAX_ANGULAR, min(MAX_ANGULAR, w))

        self.v = v
        self.w = w

        self.last_cmd_time = time.time()


    # ==================================================

    def check_stall(self):

        current = self.read_motor_current()

        speed = abs(self.v) + abs(self.w)

        now = time.time()

        if speed > 0.05 and current > STALL_CURRENT:

            if self.stall_start is None:
                self.stall_start = now

            elif now - self.stall_start > STALL_TIME:
                self.trigger_emergency()

        else:
            self.stall_start = None


    # ==================================================

    def trigger_emergency(self):

        self.emergency = True
        self.recover_time = time.time() + RECOVERY_TIME

        self.stop_motors()

        self.v = 0.0
        self.w = 0.0

        self.get_logger().error("🚨 MOTOR STALL → EMERGENCY STOP")


    # ==================================================

    def try_recover(self):

        if not self.emergency:
            return

        if time.time() > self.recover_time:

            self.emergency = False
            self.stall_start = None

            self.get_logger().info("✅ Emergency released")


    # ==================================================

    def set_left(self, speed):

        duty = int(abs(speed) * 100)
        duty = max(0, min(100, duty))

        if speed >= 0:
            lgpio.gpio_write(self.chip, IN1, 1)
            lgpio.gpio_write(self.chip, IN2, 0)
        else:
            lgpio.gpio_write(self.chip, IN1, 0)
            lgpio.gpio_write(self.chip, IN2, 1)

        lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, duty)


    def set_right(self, speed):

        duty = int(abs(speed) * 100)
        duty = max(0, min(100, duty))

        if speed >= 0:
            lgpio.gpio_write(self.chip, IN3, 1)
            lgpio.gpio_write(self.chip, IN4, 0)
        else:
            lgpio.gpio_write(self.chip, IN3, 0)
            lgpio.gpio_write(self.chip, IN4, 1)

        lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, duty)


    # ==================================================

    def stop_motors(self):

        lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, 0)
        lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, 0)

        for p in [IN1, IN2, IN3, IN4]:
            lgpio.gpio_write(self.chip, p, 0)


    # ==================================================

    def update(self):

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # -------- WATCHDOG --------

        if now - self.last_cmd_time > CMD_TIMEOUT:
            self.v = 0.0
            self.w = 0.0

        # -------- PROTECTION --------

        self.check_stall()
        self.try_recover()

        if self.emergency:
            return


        # -------- KINEMATICS --------

        v_l = self.v - (self.w * WHEEL_BASE / 2.0)
        v_r = self.v + (self.w * WHEEL_BASE / 2.0)

        max_v = max(abs(v_l), abs(v_r))

        if max_v > MAX_LINEAR:
            k = MAX_LINEAR / max_v
            v_l *= k
            v_r *= k

        left = v_l / MAX_LINEAR
        right = v_r / MAX_LINEAR


        # -------- MOTORS --------

        self.set_left(left)
        self.set_right(right)


        # -------- ODOM --------

        v = (v_r + v_l) / 2.0
        w = (v_r - v_l) / WHEEL_BASE

        self.theta += w * dt

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        self.publish_odom(v, w, now)


    # ==================================================

    def publish_odom(self, v, w, stamp):

        msg = Odometry()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        self.odom_pub.publish(msg)


    # ==================================================

    def destroy_node(self):

        self.stop_motors()
        lgpio.gpiochip_close(self.chip)

        super().destroy_node()


# ==================================================


def main():

    rclpy.init()

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
