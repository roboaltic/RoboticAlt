#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import RPi.GPIO as GPIO
import time
import math

# ================= CONFIG =================

# Motor pins (CHANGE IF NEEDED)
LEFT_IN1 = 17
LEFT_IN2 = 18
RIGHT_IN1 = 22
RIGHT_IN2 = 23
LEFT_EN = 24
RIGHT_EN = 25

PWM_FREQ = 1000

# Stall protection
STALL_TIME = 1.0        # seconds
MIN_MOVE_DIST = 0.002   # meters

# Robot params
WHEEL_RADIUS = 0.033
WHEEL_BASE = 0.16
MAX_PWM = 100

# ==========================================


class DiffDrive(Node):

    def __init__(self):
        super().__init__('diff_drive_l298n')

        # ROS
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.timer = self.create_timer(0.05, self.update)  # 20 Hz

        # Velocity
        self.v = 0.0
        self.w = 0.0

        # Odometry
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_time = time.time()

        # Stall detection
        self.last_move_time = time.time()
        self.last_x = 0.0
        self.last_y = 0.0
        self.stalled = False

        # GPIO
        self.setup_gpio()

        self.get_logger().info("DiffDrive L298N with Stall Protection Started")

    # ================= GPIO ==================

    def setup_gpio(self):

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        pins = [LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2, LEFT_EN, RIGHT_EN]

        for p in pins:
            GPIO.setup(p, GPIO.OUT)
            GPIO.output(p, GPIO.LOW)

        self.pwm_l = GPIO.PWM(LEFT_EN, PWM_FREQ)
        self.pwm_r = GPIO.PWM(RIGHT_EN, PWM_FREQ)

        self.pwm_l.start(0)
        self.pwm_r.start(0)

    # =========================================

    def cmd_callback(self, msg):

        self.v = msg.linear.x
        self.w = msg.angular.z

        # Reset stall when new command
        if abs(self.v) > 0.01 or abs(self.w) > 0.01:
            self.stalled = False

    # =========================================

    def set_motor(self, in1, in2, pwm, speed):

        speed = max(-1.0, min(1.0, speed))

        duty = abs(speed) * MAX_PWM

        if speed > 0:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        elif speed < 0:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)

        pwm.ChangeDutyCycle(duty)

    def stop_motors(self):

        self.pwm_l.ChangeDutyCycle(0)
        self.pwm_r.ChangeDutyCycle(0)

        GPIO.output(LEFT_IN1, GPIO.LOW)
        GPIO.output(LEFT_IN2, GPIO.LOW)
        GPIO.output(RIGHT_IN1, GPIO.LOW)
        GPIO.output(RIGHT_IN2, GPIO.LOW)

    # =========================================

    def check_stall(self):

        if abs(self.v) < 0.01 and abs(self.w) < 0.01:
            self.stalled = False
            return

        dx = self.x - self.last_x
        dy = self.y - self.last_y

        dist = math.sqrt(dx * dx + dy * dy)

        now = time.time()

        if dist > MIN_MOVE_DIST:

            self.last_move_time = now
            self.last_x = self.x
            self.last_y = self.y
            self.stalled = False
            return

        if now - self.last_move_time > STALL_TIME:

            if not self.stalled:
                self.get_logger().warn("MOTOR STALL DETECTED! Emergency stop")

            self.stalled = True

            self.v = 0.0
            self.w = 0.0

            self.stop_motors()

    # =========================================

    def update_odometry(self, vl, vr, dt):

        v = (vr + vl) / 2.0
        w = (vr - vl) / WHEEL_BASE

        dx = v * math.cos(self.th) * dt
        dy = v * math.sin(self.th) * dt
        dth = w * dt

        self.x += dx
        self.y += dy
        self.th += dth

    # =========================================

    def publish_odom(self, v, w):

        msg = Odometry()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        self.odom_pub.publish(msg)

    # =========================================

    def update(self):

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # Wheel speeds
        vl = self.v - self.w * WHEEL_BASE / 2.0
        vr = self.v + self.w * WHEEL_BASE / 2.0

        # Normalize
        max_v = max(abs(vl), abs(vr), 0.001)

        if max_v > 1.0:
            vl /= max_v
            vr /= max_v

        # Odometry
        self.update_odometry(vl, vr, dt)
        self.publish_odom(self.v, self.w)

        # -------- STALL CHECK --------
        self.check_stall()

        if self.stalled:
            return

        # -------- MOTORS --------
        self.set_motor(LEFT_IN1, LEFT_IN2, self.pwm_l, vl)
        self.set_motor(RIGHT_IN1, RIGHT_IN2, self.pwm_r, vr)

    # =========================================

    def destroy_node(self):

        self.stop_motors()
        GPIO.cleanup()

        super().destroy_node()


# =============================================


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
