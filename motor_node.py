#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import lgpio
import math
import time


# ================== GPIO ==================

ENA = 18   # PWM Left
IN1 = 23
IN2 = 24

ENB = 13   # PWM Right
IN3 = 5
IN4 = 6

PWM_FREQ = 20000   # 20 kHz


# ================== ROBOT PARAMS ==================

WHEEL_BASE = 0.18        # meters
MAX_SPEED = 0.25         # m/s

CMD_TIMEOUT = 1.0        # sec
STALL_TIME = 2.0         # sec


# ================================================


class DiffDriveNode(Node):

    def __init__(self):
        super().__init__('diff_drive_node')

        # ---------------- ROS ----------------

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_cb,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.timer = self.create_timer(0.05, self.update)

        # ---------------- GPIO ----------------

        self.chip = lgpio.gpiochip_open(0)

        for pin in [ENA, IN1, IN2, ENB, IN3, IN4]:
            lgpio.gpio_claim_output(self.chip, pin)

        # ---------------- STATE ----------------

        self.v = 0.0
        self.w = 0.0

        self.last_cmd_time = time.time()

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_time = time.time()

        self.stall_start = None

        self.get_logger().info('DiffDrive L298N node started')

    # ======================================================

    def limit(self, v):
        return max(min(v, MAX_SPEED), -MAX_SPEED)

    # ======================================================

    def cmd_vel_cb(self, msg):

        self.v = self.limit(msg.linear.x)
        self.w = msg.angular.z

        self.last_cmd_time = time.time()

    # ======================================================

    def set_left(self, speed):

        fwd = speed >= 0

        duty = min(max(abs(speed) / MAX_SPEED * 100, 0.0), 100.0)

        lgpio.gpio_write(self.chip, IN1, int(fwd))
        lgpio.gpio_write(self.chip, IN2, int(not fwd))

        lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, duty)

    # ======================================================

    def set_right(self, speed):

        fwd = speed >= 0

        duty = min(max(abs(speed) / MAX_SPEED * 100, 0.0), 100.0)

        lgpio.gpio_write(self.chip, IN3, int(fwd))
        lgpio.gpio_write(self.chip, IN4, int(not fwd))

        lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, duty)

    # ======================================================

    def stop(self):

        lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, 0)
        lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, 0)

    # ======================================================

    def publish_odom(self, v, w, dt):

        dx = v * math.cos(self.th) * dt
        dy = v * math.sin(self.th) * dt
        dth = w * dt

        self.x += dx
        self.y += dy
        self.th += dth

        msg = Odometry()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y

        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        self.odom_pub.publish(msg)

    # ======================================================

    def check_stall(self, v, duty):

        # Если даём газ, но долго нет движения → стоп

        if duty > 30 and abs(v) < 0.01:

            if self.stall_start is None:
                self.stall_start = time.time()

            elif time.time() - self.stall_start > STALL_TIME:

                self.get_logger().warn('STALL DETECTED! Emergency stop.')

                self.stop()
                self.v = 0.0
                self.w = 0.0

                return True
        else:
            self.stall_start = None

        return False

    # ======================================================

    def update(self):

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # -------- Watchdog --------

        if now - self.last_cmd_time > CMD_TIMEOUT:
            self.stop()
            self.v = 0.0
            self.w = 0.0
            return

        # -------- Kinematics --------

        vl = self.limit(self.v - self.w * WHEEL_BASE / 2)
        vr = self.limit(self.v + self.w * WHEEL_BASE / 2)

        duty_l = abs(vl) / MAX_SPEED * 100
        duty_r = abs(vr) / MAX_SPEED * 100

        # -------- Stall protect --------

        if self.check_stall(self.v, max(duty_l, duty_r)):
            return

        # -------- Drive --------

        self.set_left(vl)
        self.set_right(vr)

        # -------- Odom --------

        self.publish_odom(self.v, self.w, dt)

    # ======================================================

    def destroy_node(self):

        self.stop()
        lgpio.gpiochip_close(self.chip)

        super().destroy_node()


# ======================================================


def main():

    rclpy.init()

    node = DiffDriveNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
