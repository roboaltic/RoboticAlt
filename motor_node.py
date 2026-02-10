#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO
import time
import math


class DiffDriveNode(Node):

    def __init__(self):
        super().__init__('diff_drive_node')

        # ===== GPIO SETUP =====

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Motor pins (CHANGE IF NEEDED)
        self.ENA = 22
        self.IN1 = 17
        self.IN2 = 27

        self.ENB = 13
        self.IN3 = 26
        self.IN4 = 19

        pins = [
            self.ENA, self.IN1, self.IN2,
            self.ENB, self.IN3, self.IN4
        ]

        for p in pins:
            GPIO.setup(p, GPIO.OUT)

        self.pwmA = GPIO.PWM(self.ENA, 1000)
        self.pwmB = GPIO.PWM(self.ENB, 1000)

        self.pwmA.start(0)
        self.pwmB.start(0)

        # ===== ROS =====

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # ===== SAFETY =====

        self.last_cmd_time = time.time()

        self.current_linear = 0.0
        self.current_angular = 0.0

        self.emergency_stop = False

        # settings
        self.stall_timeout = 1.5     # sec
        self.min_cmd = 0.05          # m/s

        # Timer 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.safety_check
        )

        self.get_logger().info("DiffDrive node with protection started")


    # ================= MOTOR CONTROL =================

    def set_motor(self, left, right):

        # Direction LEFT
        if left >= 0:
            GPIO.output(self.IN1, GPIO.HIGH)
            GPIO.output(self.IN2, GPIO.LOW)
        else:
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.HIGH)

        # Direction RIGHT
        if right >= 0:
            GPIO.output(self.IN3, GPIO.HIGH)
            GPIO.output(self.IN4, GPIO.LOW)
        else:
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.HIGH)

        # PWM
        self.pwmA.ChangeDutyCycle(abs(left))
        self.pwmB.ChangeDutyCycle(abs(right))


    def stop_motors(self):

        self.pwmA.ChangeDutyCycle(0)
        self.pwmB.ChangeDutyCycle(0)

        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)


    # ================= CMD_VEL =================

    def cmd_callback(self, msg: Twist):

        self.last_cmd_time = time.time()

        self.current_linear = msg.linear.x
        self.current_angular = msg.angular.z

        # Reset emergency
        if abs(self.current_linear) < 0.01:
            if self.emergency_stop:
                self.get_logger().warn("Emergency stop released")

            self.emergency_stop = False


        if self.emergency_stop:
            self.stop_motors()
            return


        left, right = self.compute_wheels(
            self.current_linear,
            self.current_angular
        )

        self.set_motor(left, right)


    # ================= KINEMATICS =================

    def compute_wheels(self, v, w):

        wheel_base = 0.16   # meters
        max_pwm = 80

        left = v - w * wheel_base / 2
        right = v + w * wheel_base / 2

        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        left_pwm = left * max_pwm
        right_pwm = right * max_pwm

        return left_pwm, right_pwm


    # ================= SAFETY =================

    def safety_check(self):

        if self.emergency_stop:
            return


        now = time.time()
        dt = now - self.last_cmd_time

        moving_cmd = abs(self.current_linear) > self.min_cmd


        if moving_cmd and dt > self.stall_timeout:

            self.get_logger().error(
                "MOTOR STALL DETECTED! EMERGENCY STOP!"
            )

            self.emergency_stop = True
            self.stop_motors()


    # ================= CLEANUP =================

    def destroy_node(self):

        self.stop_motors()
        GPIO.cleanup()

        super().destroy_node()



def main(args=None):

    rclpy.init(args=args)

    node = DiffDriveNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
