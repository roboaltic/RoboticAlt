#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import lgpio
import math
import time
import sys


# ================= GPIO ==================

GPIO_CHIP = 4

ENA = 22
IN1 = 17
IN2 = 27

ENB = 13
IN3 = 26
IN4 = 19


# ================ ROBOT ==================

WHEEL_BASE = 0.09
MAX_SPEED = 0.3
PWM_FREQ = 1000


# ============== STALL ====================

STALL_TIME = 1.5
MIN_REAL_SPEED = 0.02
MIN_CMD_SPEED = 0.1


# ========================================


class DiffDrive(Node):

    def __init__(self):

        super().__init__('diff_drive_node')


        # ---------- GPIO ----------

        self.chip = lgpio.gpiochip_open(GPIO_CHIP)

        pins = [ENA, IN1, IN2, ENB, IN3, IN4]

        try:
            for p in pins:
                lgpio.gpio_claim_output(self.chip, p)

        except lgpio.error as e:
            self.get_logger().error(f"GPIO ERROR: {e}")
            sys.exit(1)


        lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, 0)
        lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, 0)


        # ---------- ROS ----------

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


        self.timer = self.create_timer(
            0.05,
            self.update_odom
        )


        # ---------- STATE ----------

        self.v = 0.0
        self.w = 0.0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_time = self.get_clock().now()

        self.last_motion = self.get_clock().now()
        self.stalled = False


        self.get_logger().info("✅ DiffDrive + Stall protection READY")


    # ======================================
    # CMD_VEL
    # ======================================

    def cmd_vel_cb(self, msg):

        if self.stalled:
            return

        self.v = self.limit(msg.linear.x)
        self.w = self.limit(msg.angular.z)

        vl = self.v - self.w * WHEEL_BASE / 2
        vr = self.v + self.w * WHEEL_BASE / 2

        self.set_left(vl)
        self.set_right(vr)


    # ======================================
    # MOTOR
    # ======================================

    def set_left(self, speed):

        fwd = speed >= 0
        duty = abs(speed) / MAX_SPEED * 100

        lgpio.gpio_write(self.chip, IN1, int(fwd))
        lgpio.gpio_write(self.chip, IN2, int(not fwd))

        lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, duty)

    def set_right(self, speed):

        # ===== Исправлено: инверсия направления =====
        fwd = speed < 0  # <--- раньше было speed >=0, теперь зеркально
        duty = abs(speed) / MAX_SPEED * 100

        lgpio.gpio_write(self.chip, IN3, int(fwd))
        lgpio.gpio_write(self.chip, IN4, int(not fwd))

        lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, duty)


    def stop(self):

        lgpio.tx_pwm(self.chip, ENA, PWM_FREQ, 0)
        lgpio.tx_pwm(self.chip, ENB, PWM_FREQ, 0)


    # ======================================
    # ODOM
    # ======================================

    def update_odom(self):

        now = self.get_clock().now()

        dt = (now - self.last_time).nanoseconds * 1e-9

        self.last_time = now

        vx = self.v
        vth = self.w

        self.x += vx * math.cos(self.th) * dt
        self.y += vx * math.sin(self.th) * dt
        self.th += vth * dt

        self.check_stall()

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)


    # ======================================
    # STALL
    # ======================================

    def check_stall(self):

        now = self.get_clock().now()

        if abs(self.v) > MIN_CMD_SPEED:

            if abs(self.v) < MIN_REAL_SPEED:

                dt = (now - self.last_motion).nanoseconds * 1e-9

                if dt > STALL_TIME:

                    self.get_logger().error("🚨 MOTOR STALL!")

                    self.stop()

                    self.stalled = True

                    return

            else:

                self.last_motion = now
                self.stalled = False


    # ======================================
    # UTILS
    # ======================================

    def limit(self, v):

        return max(min(v, MAX_SPEED), -MAX_SPEED)


    def destroy_node(self):

        self.stop()
        lgpio.gpiochip_close(self.chip)
        super().destroy_node()


# =========================================

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


if __name__ == "__main__":

    main()
