import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO


class DiffDriveL298N(Node):

    def __init__(self):
        super().__init__('diff_drive_l298n')

        # ===== PARAMETERS =====
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('pwm_frequency', 1000)

        self.max_linear = self.get_parameter(
            'max_linear_speed').value
        self.max_angular = self.get_parameter(
            'max_angular_speed').value
        self.pwm_freq = self.get_parameter(
            'pwm_frequency').value

        # ===== GPIO PINS (BCM) =====
        # L298N
        self.in1 = 17   # Motor A
        self.in2 = 27
        self.in3 = 26   # Motor B
        self.in4 = 19

        self.en_a = 22  # PWM Motor A
        self.en_b = 13  # PWM Motor B

        # ===== GPIO INIT =====
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.in3, GPIO.OUT)
        GPIO.setup(self.in4, GPIO.OUT)
        GPIO.setup(self.en_a, GPIO.OUT)
        GPIO.setup(self.en_b, GPIO.OUT)

        self.pwm_left = GPIO.PWM(self.en_a, self.pwm_freq)
        self.pwm_right = GPIO.PWM(self.en_b, self.pwm_freq)

        self.pwm_left.start(0)
        self.pwm_right.start(0)

        self.stop_motors()

        self.get_logger().info("L298N motor driver initialized")

        # ===== ROS SUB =====
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # ===== WATCHDOG =====
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self.watchdog_check)

    # ======================================================

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        # --- LIMIT INPUT ---
        linear = max(
            -self.max_linear,
            min(self.max_linear, msg.linear.x)
        )
        angular = max(
            -self.max_angular,
            min(self.max_angular, msg.angular.z)
        )

        # --- DIFF DRIVE ---
        left = linear - angular
        right = linear + angular

        left_pwm = min(abs(left) / self.max_linear * 100, 100)
        right_pwm = min(abs(right) / self.max_linear * 100, 100)

        # --- LEFT MOTOR ---
        if left >= 0:
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)

        # --- RIGHT MOTOR ---
        if right >= 0:
            GPIO.output(self.in3, GPIO.HIGH)
            GPIO.output(self.in4, GPIO.LOW)
        else:
            GPIO.output(self.in3, GPIO.LOW)
            GPIO.output(self.in4, GPIO.HIGH)

        self.pwm_left.ChangeDutyCycle(left_pwm)
        self.pwm_right.ChangeDutyCycle(right_pwm)

    # ======================================================

    def watchdog_check(self):
        dt = (self.get_clock().now() -
              self.last_cmd_time).nanoseconds / 1e9

        if dt > 0.5:
            self.stop_motors()

    def stop_motors(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.LOW)
        self.pwm_left.ChangeDutyCycle(0)
        self.pwm_right.ChangeDutyCycle(0)

    def destroy_node(self):
        self.stop_motors()
        GPIO.cleanup()
        super().destroy_node()


def main():
    rclpy.init()
    node = DiffDriveL298N()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
